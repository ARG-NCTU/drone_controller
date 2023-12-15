#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Twist


class CombinePoseCamError:
    def __init__(self):
        self.rate = float(rospy.get_param("~rate", 10.0))

        self.pose_sub = rospy.Subscriber("/pozyx_simulation/drone/pose/optim", PoseStamped, self.pose_callback)
        self.cam_error_sub = rospy.Subscriber("/drone/perception/target/error", Twist, self.cam_error_callback)
        self.controller_error_pub = rospy.Publisher("/drone/controller/target/error", Twist, queue_size=1)

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
        self.controller_error.linear.x = -self.pose.pose.position.x - 1.5
        # self.controller_error.linear.x = self.cam_error.linear.z
        self.controller_error.linear.y = -self.pose.pose.position.y
        self.controller_error.linear.z = self.cam_error.linear.z
        self.controller_error.angular.z = self.cam_error.linear.y
        self.controller_error_pub.publish(self.controller_error)
        self.pose = None
        self.cam_error = None


if __name__ == "__main__":
    rospy.init_node("combine_pose_cam_error")
    CombinePoseCamError()
    rospy.spin()
