#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16MultiArray
from mavros_msgs.msg import RCOut

def rc_callback(data):
    # Calculate the delta from 1500 for the first four channels
    deltas = [channel - 1500 for channel in data.channels[:4]]

    # Prepare to publish the deltas as Int16
    delta_msg = Int16MultiArray()
    delta_msg.data = deltas
    pub.publish(delta_msg)

if __name__ == '__main__':
    try:
        # Initialize ROS node
        rospy.init_node('rc_delta_publisher', anonymous=True)

        # Create a publisher for Int16MultiArray message on a new topic
        pub = rospy.Publisher('/propeller_data', Int16MultiArray, queue_size=10)

        # Subscribe to the /mavros/rc/out topic
        rospy.Subscriber("/mavros/rc/out", RCOut, rc_callback)

        # Keep the script running
        rospy.spin()
    except rospy.ROSInterruptException:
        pass