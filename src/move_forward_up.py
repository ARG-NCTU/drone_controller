#! /usr/bin/env python

import rospy
from behavior_tree_msgs.msg import Active, Status
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Float32, Float64

FAIL = 0
RUNNING = 1
SUCCESS = 2


class DroneController:
    def __init__(self):
        rospy.init_node("drone_move_forward_up")
        self.publish_rate = rospy.get_param("~publish_rate", 10)

        self.kp_linear = rospy.get_param("~kp_linear", 2)
        self.ki_linear = rospy.get_param("~ki_linear", 0.1)
        self.kd_linear = rospy.get_param("~kd_linear", 0.0)

        self.kp_angular = rospy.get_param("~kp_angular", 1)
        self.ki_angular = rospy.get_param("~ki_angular", 0.1)
        self.kd_angular = rospy.get_param("~kd_angular", 0.0)

        self.target_x_speed = rospy.get_param("~target_linear_speed", 1.0)
        self.target_z_speed = rospy.get_param("~target_angular_speed", 0.3)

        self.drone_move_forward_up_status_pub = rospy.Publisher("/drone_move_forward_up_status", Status, queue_size=10)
        self.drone_command_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)
        rospy.Subscriber("/drone_move_forward_up_active", Active, self.active_callback)
        rospy.Subscriber("/mavros/local_position/velocity_body", TwistStamped, self.twist_callback)

        self.prev_error_x = 0
        self.integral_x = 0
        self.prev_error_z = 0
        self.integral_z = 0

        self.current_speed_x = 0
        self.current_speed_z = 0

        self.active = False

        rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.timer_callback)

    def twist_callback(self, msg):
        self.current_speed_x = msg.twist.linear.x
        self.current_speed_z = msg.twist.angular.z

    def target_linear_speed_callback(self, msg):
        self.target_x_speed = msg.data

    def target_angular_speed_callback(self, msg):
        self.target_z_speed = msg.data

    def pid_control(self, target, current, prev_error, integral, kp, ki, kd):
        error = target - current
        integral += error * (1.0 / self.publish_rate)
        derivative = (error - prev_error) / (1.0 / self.publish_rate)

        output = kp * error + ki * integral + kd * derivative
        prev_error = error

        return output, prev_error, integral

    def timer_callback(self, event):
        if not self.active:
            x_vel = 0
            z_vel = 0
            # Set I to zero when not active
            self.integral_x = 0
            self.integral_z = 0
            return

        x_vel, self.prev_error_x, self.integral_x = self.pid_control(
            self.target_x_speed,
            self.current_speed_x,
            self.prev_error_x,
            self.integral_x,
            self.kp_linear,
            self.ki_linear,
            self.kd_linear,
        )

        z_vel, self.prev_error_z, self.integral_z = self.pid_control(
            self.target_z_speed,
            self.current_speed_z,
            self.prev_error_z,
            self.integral_z,
            self.kp_angular,
            self.ki_angular,
            self.kd_angular,
        )
        
        vel = Twist()
        vel.linear.x = x_vel
        vel.linear.z = z_vel
        self.drone_command_pub.publish(vel)

        print("X Vel: {}, Z Vel: {}          ".format(x_vel, z_vel))

    def active_callback(self, msg):
        self.active = msg.active
        status = Status()
        status.id = msg.id
        status.status = SUCCESS
        self.drone_move_forward_up_status_pub.publish(status)


if __name__ == "__main__":
    DroneController()
    rospy.spin()
