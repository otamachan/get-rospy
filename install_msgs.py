# -*- codeing: utf-8 -*-
from __future__ import print_function
import get_rospy
MESSAGES = [
    # List of tuples.
    # A tuple consists of
    #  - repository(GitHub)
    #  - version or branch
    #  - path(in the repository)
    #  - depend message packages(list)
    # repository and version can be an empty string, if message files are
    # already insatlled under site-packages.
    ('ros/std_msgs', '0.5.11', '', []),
    ('ros/ros_comm', '1.12.7', 'clients/roscpp', []),
    ('ros/ros_comm_msgs', '1.11.2', 'rosgraph_msgs', ['std_msgs']),
]
get_rospy.install_messages(MESSAGES)
print("done")
