# -*- codeing: utf-8 -*-
import get_rospy
MESSAGES = [
    ('ros/std_msgs', '0.5.10', '', []),
    ('ros/ros_comm', '1.12.0', 'clients/roscpp', []),
    ('ros/ros_comm_msgs', '1.11.2', 'rosgraph_msgs', ['std_msg']),
]
get_rospy.install_messages(MESSAGES)
