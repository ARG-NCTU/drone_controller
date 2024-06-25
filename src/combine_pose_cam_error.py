#!/usr/bin/env python
import rospy
from behavior_tree_msgs.msg import Active, Status
from geometry_msgs.msg import PoseStamped, Twist

FAIL = 0
RUNNING = 1
SUCCESS = 2


class CombinePoseCamError:
    def __init__(self):
        self.rate = float(rospy.get_param("~rate", 10.0))

        self.pose_sub = rospy.Subscriber("/pozyx_simulation/drone/pose/ground_truth", PoseStamped, self.pose_callback)
        self.cam_error_sub = rospy.Subscriber("/drone/perception/target/error", Twist, self.cam_error_callback)
        self.controller_error_pub = rospy.Publisher("/drone/controller/target/error", Twist, queue_size=1)

        self.drone_move_forward_status_pub = rospy.Publisher("/drone_move_forward_status", Status, queue_size=10)
        rospy.Subscriber("/drone_move_forward_active", Active, self.active_callback)

        self.active = False

        self.target_x = -1.6
        self.pose = None
        self.cam_error = None
        self.controller_error = Twist()
        self.timer = rospy.Timer(rospy.Duration(1 / self.rate), self.timer_callback)

    def pose_callback(self, pose):
        self.pose = pose

    def cam_error_callback(self, cam_error):
        self.cam_error = cam_error

    def timer_callback(self, event):
        if self.pose is None or self.cam_error is None:
            return
        if self.active:
            if self.target_x < -0.2:
                self.target_x += 0.06
            else:
                self.target_x = -1.2
        # else:
            # self.target_x = -2.0
        print("target x: {}".format(self.target_x))

        self.controller_error.linear.x = -self.pose.pose.position.x + self.target_x
        # self.controller_error.linear.x = self.cam_error.linear.z
        self.controller_error.linear.y = -self.pose.pose.position.y
        self.controller_error.linear.z = self.cam_error.linear.z
        self.controller_error.angular.z = self.cam_error.linear.y + 0.2
        self.controller_error_pub.publish(self.controller_error)
        self.pose = None
        self.cam_error = None

    def active_callback(self, msg):
        self.active = msg.active
        status = Status()
        status.id = msg.id
        status.status = SUCCESS
        self.drone_move_forward_status_pub.publish(status)


if __name__ == "__main__":
    rospy.init_node("combine_pose_cam_error")
    CombinePoseCamError()
    rospy.spin()
