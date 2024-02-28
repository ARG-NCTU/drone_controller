#! /usr/bin/env python

import numpy as np
import rospy
import tf.transformations as tft
from behavior_tree_msgs.msg import Active, Status
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

FAIL = 0
RUNNING = 1
SUCCESS = 2


class TrackWAMVUWBPose:
    def __init__(self):
        node_name = rospy.get_name().split("/")[-1]
        node_name = "track_wamv_uwb_pose"
        self.track_gps_finish_success_pub = rospy.Publisher("/{}_finish_success".format(node_name), Bool, queue_size=1)
        self.behavior_active_sub = rospy.Subscriber(
            "/{}_active".format(node_name), Active, self.behavior_active_callback
        )
        self.behavior_status_pub = rospy.Publisher("/{}_status".format(node_name), Status, queue_size=1)
        # self.wamv_pose_to_local_sub = rospy.Subscriber(
        #     "/wamv/localization_gps_imu/pose", PoseStamped, self.wamv_pose_to_local_callback
        # )
        self.drone_pose_to_wamv_sub = rospy.Subscriber(
            "/pozyx_simulation/drone/pose/ground_truth", PoseStamped, self.drone_pose_to_wamv_callback
        )
        self.drone_pose_to_local_sub = rospy.Subscriber(
            "mavros/local_position/pose", PoseStamped, self.drone_pose_to_local_callback
        )
        self.setpoint_position_local_pub = rospy.Publisher(
            "/mavros/setpoint_position/local", PoseStamped, queue_size=10
        )
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

        # self.wamv_pose_to_local = np.identity(4)
        # self.wamv_pose_to_local_received = False

        self.drone_pose_to_wamv = np.identity(4)
        self.drone_pose_to_wamv_received = False
        self.drone_pose_to_wamv_msg = PoseStamped()

        self.drone_pose_to_local = np.identity(4)
        self.drone_pose_to_local_received = False

        self.active = Active()
        self.track_gps_finish_success = Bool()

    # def wamv_pose_to_local_callback(self, msg):
    #     self.wamv_pose_to_local_msg = msg
    #     self.wamv_pose_to_local = self.pose_stamped_to_matrix(msg)
    #     self.wamv_pose_to_local_received = True

    def drone_pose_to_wamv_callback(self, msg):
        self.drone_pose_to_wamv_msg = msg
        self.drone_pose_to_wamv = self.pose_stamped_to_matrix(msg)
        self.drone_pose_to_wamv_received = True

    def drone_pose_to_local_callback(self, msg):
        self.drone_pose_to_local = self.pose_stamped_to_matrix(msg)
        self.drone_pose_to_local_received = True

    def behavior_active_callback(self, msg):
        self.active = msg

    def timer_callback(self, event):
        # if not self.wamv_pose_to_local_received:
        #     rospy.logwarn_throttle(1, "Waiting for wamv pose to local")
        #     return
        if not self.drone_pose_to_wamv_received:
            rospy.logwarn_throttle(1, "Waiting for drone pose to wamv")
            return
        if not self.drone_pose_to_local_received:
            rospy.logwarn_throttle(1, "Waiting for drone pose to local")
            return

        wamv_to_drone = np.zeros_like(self.drone_pose_to_wamv)

        if not np.allclose(self.drone_pose_to_wamv, np.identity(4)):
            wamv_to_drone = np.linalg.inv(self.drone_pose_to_wamv)

        # Condition
        r, theta, z = self.calculate_cylindrical_corrdinate(
            self.drone_pose_to_wamv[0, 3], self.drone_pose_to_wamv[1, 3], self.drone_pose_to_wamv[2, 3]
        )
        yaw = tft.euler_from_matrix(self.drone_pose_to_wamv)[2]
        # print("r: {}, theta: {}, z: {}, yaw: {}".format(r, theta, z, yaw))
        # print(self.track_gps_finish_success.data)
        if abs(r - 3.0) < 5 and abs(theta) > 2.6 and abs(z - 2.0) < 2.5 and abs(yaw) < 0.45:
            self.track_gps_finish_success.data = True
        else:
            self.track_gps_finish_success.data = False

        if abs(r - 3.0) >= 5:
            rospy.loginfo("r: {}".format(r))
        if abs(theta) <= 2.6:
            rospy.loginfo("theta: {}".format(theta))
        if abs(z - 2.0) >= 2.5:
            rospy.loginfo("z: {}".format(z))
        if abs(yaw) >= 0.45:
            rospy.loginfo("yaw: {}".format(yaw))
        self.track_gps_finish_success_pub.publish(self.track_gps_finish_success)

        # Action
        if self.active.active:
            status = Status()
            status.id = self.active.id
            status.status = RUNNING
            self.behavior_status_pub.publish(status)
            if not np.allclose(self.drone_pose_to_wamv, np.identity(4)):
                if not np.allclose(self.drone_pose_to_local, np.identity(4)):
                    wamv_to_drone[0, 3] += -3.5
                    wamv_to_drone[2, 3] += 2.0
                    wamv_to_local = np.dot(self.drone_pose_to_local, wamv_to_drone)
                    wamv_to_local_msg = self.matrix_to_pose_stamped(wamv_to_local, "map")
                    self.setpoint_position_local_pub.publish(wamv_to_local_msg)
                else:
                    rospy.loginfo_throttle(1.0, "Drone pose to local is not available.")
            else:
                rospy.loginfo_throttle(1.0, "Drone pose to WAMV is not available.")
        else:
            status = Status()
            status.id = self.active.id
            status.status = FAIL
            self.behavior_status_pub.publish(status)

    def pose_stamped_to_matrix(self, pose):
        matrix = tft.quaternion_matrix(
            (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)
        )
        matrix[0, 3] = pose.pose.position.x
        matrix[1, 3] = pose.pose.position.y
        matrix[2, 3] = pose.pose.position.z
        return matrix

    def matrix_to_pose_stamped(self, matrix, frame_id):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = frame_id
        pose.pose.position.x = matrix[0, 3]
        pose.pose.position.y = matrix[1, 3]
        pose.pose.position.z = matrix[2, 3]
        q = tft.quaternion_from_matrix(matrix)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def calculate_cylindrical_corrdinate(self, x, y, z):
        r = np.sqrt(x**2 + y**2)
        theta = np.arctan2(y, x)
        return r, theta, z


if __name__ == "__main__":
    rospy.init_node("track_wamv_uwb_pose")
    track_wamv_uwb_pose = TrackWAMVUWBPose()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
