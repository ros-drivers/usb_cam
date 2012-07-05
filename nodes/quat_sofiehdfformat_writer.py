#!/usr/bin/env python
import roslib; roslib.load_manifest('ros_sofiehdfformat')
import rospy
from geometry_msgs.msg import QuaternionStamped
def callback(data):
    rospy.loginfo(rospy.get_name()+"Received Quaternion %s",data.data)

def listener():
    rospy.init_node('sofiehdfformatwriter', anonymous=True)
    rospy.Subscriber("quat_sofiehdfformat", QuaternionStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
