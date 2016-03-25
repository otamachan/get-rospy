# ``rospy`` installer for ``Pythonista``
## Install

Type following commands in the console.
```python
>>> import requests; exec requests.get('http://bit.ly/1Rq7Zi1').text
```

## Example

Here is a very simple rospy example.
```python
import os
import rospy
os.environ['ROS_MASTER_URI'] = 'http://192.169.x.x:11311'
os.environ['ROS_IP'] = '192.168.y.y'
os.environ['ROS_PYTHON_LOG_CONFIG_FILE'] = '' # without this, init_node will fail

rospy.init_node('test')
pub = rospy.Publisher('test', std_msgs.msg.Int16, queue_size=1)
rate = rospy.Rate(1)
while not rospy.is_shutdown():
    pub.publish(3)
    rate.sleep()
```

## Add extra messages

1. Download ``install_msgs.py``

    ```python
    >>> import requests; open('install_msgs.py', 'w').write(requests.get('http://bit.ly/1q5X8mH').text)
    ```
2. Edit ``MESSAGES`` in ``install_msgs.py`` .
3. Run ``install_msgs.py`` .
