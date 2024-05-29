#!/usr/bin/env python3
import time

import numpy as np
import rospy
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import PoseStamped, Twist
from stable_baselines3 import PPO
from std_msgs.msg import Bool, Float64, Float64MultiArray
from std_srvs.srv import Trigger
from tf.transformations import euler_from_quaternion

HELIPAD_COLLISION = "wamv::wamv/base_link::wamv/base_link_fixed_joint_lump__helipad_plate_collision_collision"
CYLINDER_COLLISION = "wamv::wamv/base_link::wamv/base_link_fixed_joint_lump__cylinder_collision_collision_1"


class LandWAMVRange:
    def __init__(self):
        self.model = PPO.load(
            "/home/argrobotx/robotx-2022/rl/ppo/logs/uav-range-land-ppo-2024-05-18-17-34-33/uav-range-land-ppo-2024-05-18-17-34-33_5120000000_steps.zip",
            device="cuda",
        )
        print("Model loaded")

        self.active = True
        self.flying = True

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

        self.ctl_factor = {"x": 1.0, "y": 1.0, "z": 0.5, "yaw": 1.0}

        self.goal = np.array([0, 0, 0.5, 0.0])
        self.goal_range = self.cal_drone_pose_to_uwb_range(self.goal)

        self.obs_frame_len = 12
        self.uwb_measurement = np.zeros(self.obs_frame_len)

        rospy.Subscriber("/land_wamv_range/bool", Bool, self.cb_active)

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
        self.mavros_cmd_command = rospy.ServiceProxy("/drone/kill", Trigger)
        self.drone_ver_rod_cmd = rospy.Publisher(
            "/drone/vertical_rod_joint_position_controller/command", Float64, queue_size=1
        )
        self.bumper_sub = rospy.Subscriber("/drone/bumper_states", ContactsState, self.cb_bumper)
        self.pose_sub = rospy.Subscriber("/pozyx_simulation/drone/pose/ground_truth", PoseStamped, self.cb_pose)
        self.goal_sub = rospy.Subscriber("/land_wamv_uwb_range/goal", PoseStamped, self.cb_goal)
        self.goal_pub = rospy.Publisher("/land_wamv_uwb_range/goal/state", PoseStamped, queue_size=1)
        self.twist_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
        self.pub_timer = rospy.Timer(rospy.Duration(0.05), self.cb_pub_timer)

    def cb_active(self, msg):
        self.active = msg.data

    def cb_bumper(self, msg):
        return
        for state in msg.states:
            if state.collision2_name == HELIPAD_COLLISION:
                print("Helipad collision")
                self.goal[2] = 1.0

            if state.collision2_name == CYLINDER_COLLISION:
                print("Cylinder collision")
                self.mavros_cmd_command()
                self.flying = False
                return

    def cb_pose(self, msg):
        return
        position = msg.pose.position

        self.ctl_factor["x"] = min(1.0, abs(position.x))
        self.ctl_factor["y"] = min(1.0, abs(position.y))
        self.ctl_factor["z"] = min(0.5, abs(position.z))

        if abs(position.x) < 0.35 and abs(position.y) < 0.3:
            if self.goal[2] > 0.5:
                # self.goal[2] -= 0.002
                self.goal_range = self.cal_drone_pose_to_uwb_range(self.goal)

        # if abs(position.x) < 0.5 and abs(position.y) < 0.5:
        #     self.drone_ver_rod_cmd.publish(Float64(data=-0.2))

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
        twist.linear.x = action[0] * self.ctl_factor["x"]
        twist.linear.y = action[1] * self.ctl_factor["y"]
        twist.linear.z = action[2] * self.ctl_factor["z"]
        twist.angular.z = action[3] * self.ctl_factor["yaw"]
        if self.active:
            self.twist_pub.publish(twist)

            # print("State: ", state)
            print("Goal: ", self.goal)
            print("Action: ", action)
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "wamv/uwb/origin"
        goal.pose.position.x = self.goal[0]
        goal.pose.position.y = self.goal[1]
        goal.pose.position.z = self.goal[2]
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)


if __name__ == "__main__":
    print("Starting UAV Eval")
    start = time.time()
    rospy.init_node("land_wamv_uwb_range")
    print(f"Time to init node: {time.time() - start}")
    np.set_printoptions(precision=3, suppress=True)
    uav_eval = LandWAMVRange()
    rospy.spin()
