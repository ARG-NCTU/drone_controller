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


class TrackWAMVGPSPose:
    def __init__(self):
        node_name = rospy.get_name().split("/")[-1]
        node_name = "track_wamv_gps_pose"
        self.track_gps_finish_success_pub = rospy.Publisher("/{}_finish_success".format(node_name), Bool, queue_size=1)
        self.behavior_active_sub = rospy.Subscriber(
            "/{}_active".format(node_name), Active, self.behavior_active_callback
        )
        self.behavior_status_pub = rospy.Publisher("/{}_status".format(node_name), Status, queue_size=1)
        self.wamv_pose_to_local_sub = rospy.Subscriber(
            "/wamv/localization_gps_imu/pose", PoseStamped, self.wamv_pose_to_local_callback
        )
        self.drone_pose_to_local_sub = rospy.Subscriber(
            "mavros/local_position/pose", PoseStamped, self.drone_pose_to_local_callback
        )

        self.setpoint_position_local_pub = rospy.Publisher(
            "/mavros/setpoint_position/local", PoseStamped, queue_size=10
        )
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

        self.wamv_pose_to_local = np.identity(4)
        self.wamv_pose_to_local_msg = PoseStamped()
        self.wamv_pose_to_local_received = False

        self.drone_pose_to_local = np.identity(4)
        self.drone_pose_to_local_received = False

        self.active = Active()
        self.track_gps_finish_success = Bool()

    def wamv_pose_to_local_callback(self, msg):
        self.wamv_pose_to_local_msg = msg
        self.wamv_pose_to_local = self.pose_stamped_to_matrix(msg)
        self.wamv_pose_to_local_received = True

    def drone_pose_to_local_callback(self, msg):
        self.drone_pose_to_local = self.pose_stamped_to_matrix(msg)
        self.drone_pose_to_local_received = True

    def behavior_active_callback(self, msg):
        self.active = msg

    def timer_callback(self, event):
        if not self.wamv_pose_to_local_received:
            rospy.logwarn_throttle(1, "Waiting for wamv pose to local")
            return
        if not self.drone_pose_to_local_received:
            rospy.logwarn_throttle(1, "Waiting for drone pose to local")
            return

        drone_pose_to_wamv = np.dot(np.linalg.inv(self.wamv_pose_to_local), self.drone_pose_to_local)
        drone_pose_to_wamv = self.matrix_to_pose_stamped(drone_pose_to_wamv, "wamv/base_link")

        print("Distance: {}".format(self.calculate_distance((drone_pose_to_wamv.pose.position.x, drone_pose_to_wamv.pose.position.y))))
        if self.calculate_distance((drone_pose_to_wamv.pose.position.x, drone_pose_to_wamv.pose.position.y)) < 10.0:
            self.track_gps_finish_success.data = True
        else:
            self.track_gps_finish_success.data = False

        self.track_gps_finish_success_pub.publish(self.track_gps_finish_success)

        if self.active.active:
            status = Status()
            status.id = self.active.id
            status.status = RUNNING
            self.behavior_status_pub.publish(status)
            self.wamv_pose_to_local_msg.pose.position.z += 5.0
            self.setpoint_position_local_pub.publish(self.wamv_pose_to_local_msg)
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

    def calculate_distance(self, p1, p2=(0, 0)):
        return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


if __name__ == "__main__":
    rospy.init_node("track_wamv_gps_pose")
    track_wamv_gps_pose = TrackWAMVGPSPose()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
