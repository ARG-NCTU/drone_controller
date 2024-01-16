#! /usr/bin/env python3
import math
import os
import time

os.environ["TF_CPP_MIN_LOG_LEVEL"] = "3"
import numpy as np
import rospy
import tensorflow as tf
from behavior_tree_msgs.msg import Active, Status
from geometry_msgs.msg import PoseStamped, Twist
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32

FAIL = 0
RUNNING = 1
SUCCESS = 2


class PokingBotInference(object):
    def __init__(self):
        super().__init__()
        self.max_dis = 10  # meters
        self.laser_n = 4
        self.pos_n = 10
        self.frame = rospy.get_param("~frame", "map")
        self.action_scale = {
            "linear": rospy.get_param("~linear_scale", 0.33),
            "angular": rospy.get_param("~angular_scale", 0.4),
        }

        self.goal = None
        self.pos_track = None
        self.state_stack, self.state_label_stack = None, None
        self.last_pos = None
        self.vel_ratio = 0

        # network
        gpu = tf.config.experimental.list_physical_devices("GPU")
        tf.config.experimental.set_memory_growth(gpu[0], True)
        model_name = rospy.get_param("~model_name", "PokingBot_yellow_thesis")
        model_path = (
            "/home/argrobotx/robotx-2022/catkin_ws/src/drone_controller/Models/" + model_name + "/snapshots/policy"
        )
        self.policy_network = tf.saved_model.load(model_path)

        self.active = Active()
        self.active_prev = Active()
        self.success = Bool()

        # pub cmd
        self.vx_pub = rospy.Publisher("/drone/controller/linear_x/output", Float32, queue_size=1)
        self.wz_pub = rospy.Publisher("/drone/controller/angular_z/output", Float32, queue_size=1)
        self.pub_cmd = rospy.Publisher("husky_velocity_controller/cmd_vel", Twist, queue_size=1)
        self.pub_poking = rospy.Publisher("stick_move_joint_position_controller/command", Float32, queue_size=1)

        # subscriber, timer
        # self.sub_joy = rospy.Subscriber("joy_teleop/joy", Joy, self.cb_joy, queue_size=1)
        self.sub_goal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.cb_goal, queue_size=1)
        self.sub_odom = rospy.Subscriber("truth_map_posestamped", PoseStamped, self.cb_odom, queue_size=1)
        self.sub_laser = rospy.Subscriber("/RL/scan_label", LaserScan, self.cb_laser, queue_size=1)

        node_name = "latch"
        self.behavior_active_sub = rospy.Subscriber(
            "/{}_active".format(node_name), Active, self.behavior_active_callback
        )
        self.behavior_status_pub = rospy.Publisher("/{}_status".format(node_name), Status, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.inference)

    def behavior_active_callback(self, msg):
        self.active = msg

    def scale_pose(self, value):
        if value > 0:
            return math.log(1 + value)
        elif value < 0:
            return -math.log(1 + abs(value))

    def cb_goal(self, msg):
        if msg.header.frame_id != self.frame:
            self.goal = None
            return

        self.goal = np.array([msg.pose.position.x, msg.pose.position.y])

    def cb_odom(self, msg):
        if self.goal is None:
            self.pos_track = None
            return

        # caculate angle diff
        new_pos = np.array([msg.pose.position.x, msg.pose.position.y])
        diff = self.goal - new_pos
        r = R.from_quat(
            [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        )
        yaw = r.as_euler("zyx")[0]
        angle = math.atan2(diff[1], diff[0]) - yaw
        if angle >= np.pi:
            angle -= 2 * np.pi
        elif angle <= -np.pi:
            angle += 2 * np.pi

        # update pose tracker
        diff = np.array([self.scale_pose(v) for v in diff])
        track_pos = np.append(diff, angle)
        if self.pos_track is None:
            self.pos_track = np.tile(track_pos, (self.pos_n, 1))
        else:
            self.pos_track[:-1] = self.pos_track[1:]
            self.pos_track[-1] = track_pos
        self.last_pos = new_pos

    def cb_laser(self, msg):
        scan = np.array(msg.ranges)
        scan = np.clip(scan, 0, self.max_dis)
        intensities = np.array(msg.intensities)

        if self.state_stack is None:
            self.state_stack = np.tile(scan, (4, 1))
            self.state_label_stack = np.tile(intensities, (4, 1))  # self.state_num=4
        else:
            self.state_stack[:-1] = self.state_stack[1:]
            self.state_stack[-1] = scan
            self.state_label_stack[:-1] = self.state_label_stack[1:]
            self.state_label_stack[-1] = intensities

    def inference(self, event):
        self.goal = np.array([-1, 0])
        if self.pos_track is None:
            rospy.loginfo("no pos_track")
            return
        if (self.state_stack is None) or (self.state_label_stack is None):
            rospy.loginfo("no laser_stack")
            return

        # reshape
        laser = self.state_stack.reshape(-1)
        label = self.state_label_stack.reshape(-1)
        track = self.pos_track.reshape(-1)
        state = laser
        state = np.append(state, label)
        state = np.append(state, track)

        state = tf.convert_to_tensor([state], dtype=tf.float32)
        action = self.policy_network(state)[0].numpy()
        cmd = Twist()
        cmd.linear.x = action[0] * self.action_scale["linear"]
        cmd.angular.z = action[1] * self.action_scale["angular"]

        if self.active.active:
            status = Status()
            status.id = self.active.id
            status.status = RUNNING
            self.behavior_status_pub.publish(status)

            self.vx_pub.publish(action[0] * 0.1)
            self.wz_pub.publish(action[1] * 0.0)
            self.pub_cmd.publish(cmd)
            self.pub_poking.publish(action[2])
            if not self.active_prev.active:
                print("Active")
        else:
            status = Status()
            status.id = self.active.id
            status.status = FAIL
            self.behavior_status_pub.publish(status)

            if self.active_prev.active:
                self.vx_pub.publish(0.0)
                self.wz_pub.publish(0.0)
                self.pub_cmd.publish(Twist())
                self.pub_poking.publish(0.0)
                print("Inactive")
        self.active_prev = self.active


if __name__ == "__main__":
    rospy.init_node("PokingBotInference")
    pokingbot = PokingBotInference()
    rospy.spin()
