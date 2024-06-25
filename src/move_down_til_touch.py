#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import PoseStamped, Twist

HELIPAD_COLLISION = "wamv::wamv/base_link::wamv/base_link_fixed_joint_lump__helipad_plate_collision_collision"
CYLINDER_COLLISION = "wamv::wamv/base_link::wamv/base_link_fixed_joint_lump__cylinder_collision_collision_1"


class DroneControlNode:
    def __init__(self):
        rospy.init_node("drone_control_node", anonymous=True)

        self.HELIPAD_COLLISION = (
            "wamv::wamv/base_link::wamv/base_link_fixed_joint_lump__helipad_plate_collision_collision"
        )
        self.target_height = 1.0
        self.descent_speed = -0.5
        self.ascent_speed = 1.0

        self.current_height = 0.0
        self.flying = False

        self.pose_sub = rospy.Subscriber("/pozyx_simulation/drone/pose/ground_truth", PoseStamped, self.pose_callback)
        self.bumper_sub = rospy.Subscriber("/drone/bumper_states", ContactsState, self.bumper_callback)

        self.cmd_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)

        rospy.spin()

    def pose_callback(self, msg):
        self.current_height = msg.pose.position.z

        if not self.flying:
            self.descend()

    def bumper_callback(self, msg):
        for state in msg.states:
            if state.collision2_name == HELIPAD_COLLISION:
                print("Helipad collision")
                if self.flying:
                    self.goal[2] = 1.0

            if state.collision2_name == CYLINDER_COLLISION:
                print("Cylinder collision")
                self.mavros_cmd_command()
                self.flying = False
                return

    def descend(self):
        cmd = Twist()
        cmd.linear.z = self.descent_speed
        self.cmd_vel_pub.publish(cmd)
        rospy.loginfo("Descending")

    def ascend(self):
        self.flying = True
        while self.current_height < self.target_height:
            cmd = Twist()
            cmd.linear.z = self.ascent_speed
            self.cmd_vel_pub.publish(cmd)
            rospy.loginfo("Ascending to target height")
            rospy.sleep(0.1)

        cmd = Twist()
        cmd.linear.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        rospy.loginfo("Reached target height")


if __name__ == "__main__":
    try:
        DroneControlNode()
    except rospy.ROSInterruptException:
        pass
