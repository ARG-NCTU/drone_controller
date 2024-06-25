#!/usr/bin/env python3
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from stable_baselines3 import PPO
from std_msgs.msg import Bool, Float64MultiArray
from tf.transformations import euler_from_quaternion


class TrackWAMVRange:
    def __init__(self):
        self.model = PPO.load(
            "/home/argrobotx/robotx-2022/rl/ppo/logs/uav-range-track-ppo-2024-05-20-19-40-08/uav-range-track-ppo-2024-05-20-19-40-08_184320000_steps.zip",
            device="cuda",
        )
        print("Model loaded")

        self.active = True

        self.uwb_tag_pose = np.array([[0.5, 0.0, 0.0], [-0.5, 0.0, 0.0]])
        self.uwb_anchor_pose = np.array(
            [
                [-0.66, 0.442, 0.0],
                [0.66, -0.442, 0.0],
                [-0.66, -0.442, 0.0],
                [0.66, 0.442, 0.0],
                [0.0, -0.442, 0.24],
                [0.0, 0.442, 0.24],
            ]
        )

        self.goal = np.array([-3.0, 0.0, -0.5, 0.0])
        self.goal_range = self.cal_drone_pose_to_uwb_range(self.goal)

        self.obs_frame_len = 12
        self.uwb_measurement = np.zeros(self.obs_frame_len)

        rospy.Subscriber("/track_wamv_range/bool", Bool, self.cb_active)

        self.uwb_sub = []
        for i in range(2):
            self.uwb_sub.append(
                rospy.Subscriber(
                    "/pozyx_simulation/uwb{}/distances".format(i),
                    Float64MultiArray,
                    self.update_uwb_range,
                    i * 6,
                )
            )
        self.goal_sub = rospy.Subscriber("/track_wamv_range/goal", PoseStamped, self.cb_goal)
        self.twist_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
        self.pub_timer = rospy.Timer(rospy.Duration(0.05), self.cb_pub_timer, reset=True)

    def cb_active(self, msg):
        self.active = msg.data

    def cb_goal(self, msg):
        self.goal = np.array(
            [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
                euler_from_quaternion(
                    (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
                )[2],
            ]
        )
        self.goal_range = self.cal_drone_pose_to_uwb_range(self.goal)

    def update_uwb_range(self, msg, offset):
        self.uwb_measurement[offset : offset + 6] = np.array(msg.data[:6]) / 1000.0 / 50.0

    def cal_drone_pose_to_uwb_range(self, drone_pose):
        assert drone_pose.shape == (4,)

        # Extract drone position and yaw angle
        x, y, z, theta = drone_pose
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)

        # 4x4 Transformation matrix from drone frame to world frame
        transformation_matrix = np.array(
            [[cos_theta, -sin_theta, 0, x], [sin_theta, cos_theta, 0, y], [0, 0, 1, z], [0, 0, 0, 1]]
        )

        # Calculate distances
        distances = []
        for tag in self.uwb_tag_pose:
            # Transform tag position to world frame (using homogeneous coordinates)
            tag_homogeneous = np.append(tag, 1)
            tag_world = np.dot(transformation_matrix, tag_homogeneous)[:3]

            for anchor in self.uwb_anchor_pose:
                # Euclidean distance between tag and anchor
                distance = np.linalg.norm(tag_world - anchor)
                distances.append(distance)

        return np.array(distances) / 50.0

    def cb_pub_timer(self, event):
        state = np.concatenate((self.goal_range, self.uwb_measurement))
        action, _ = self.model.predict(state, deterministic=True)
        twist = Twist()
        twist.linear.x = action[0] * 1.0
        twist.linear.y = action[1] * 1.0
        twist.linear.z = action[2] * 1.0
        twist.angular.z = action[3] * 1.0
        if self.active:
            self.twist_pub.publish(twist)
            print("State: ", state)
            print("Action: ", action)


if __name__ == "__main__":
    print("Starting UAV Eval")
    start = time.time()
    rospy.init_node("track_wamv_range")
    print(f"Time to init node: {time.time() - start}")
    np.set_printoptions(precision=3, suppress=True)
    uav_eval = TrackWAMVRange()
    rospy.spin()
