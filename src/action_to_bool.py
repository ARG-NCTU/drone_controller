#!/usr/bin/env python

import rospy
from behavior_tree_msgs.msg import Active, Status
from std_msgs.msg import Bool


class ActionToBool:
    def __init__(self, action_names):
        self.bool_publishers = {}
        self.status_publishers = {}

        for action_name in action_names:
            bool_pub = rospy.Publisher(action_name + "/bool", Bool, queue_size=10)
            status_pub = rospy.Publisher(action_name + "_status", Status, queue_size=10)

            self.bool_publishers[action_name] = bool_pub
            self.status_publishers[action_name] = status_pub

            rospy.Subscriber(action_name + "_active", Active, self.create_callback(action_name))

    def create_callback(self, action_name):
        def callback(data):
            bool_msg = Bool()
            bool_msg.data = data.active
            self.bool_publishers[action_name].publish(bool_msg)

            status_msg = Status()
            status_msg.id = data.id
            status_msg.status = Status.SUCCESS if data.active else Status.FAILURE
            self.status_publishers[action_name].publish(status_msg)

        return callback


if __name__ == "__main__":
    rospy.init_node("multi_active_to_bool_converter")

    action_names = rospy.get_param("~action_names", ["land_wamv_range", "track_wamv_range"])

    if not action_names:
        rospy.logerr("No topics provided. Shutting down.")
        rospy.signal_shutdown("No topics provided.")

    converter = ActionToBool(action_names)

    rospy.spin()
