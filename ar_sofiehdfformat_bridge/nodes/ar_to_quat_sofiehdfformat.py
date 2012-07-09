#!/usr/bin/env python
import roslib; roslib.load_manifest('ar_sofiehdfformat')
import rospy
#import ccny_vision.ar_pose.msg;

from geometry_msgs.msg import QuaternionStamped
from ar_pose.msg import ARMarker

def listener():
    print 'Starting AR_TO_QUAT node:'
    pub = rospy.Publisher('quat_sofiehdfformat', QuaternionStamped)
    def callback(data):
        orientation = data.pose.pose.orientation
        quat = QuaternionStamped()
        quat.header = data.header
        quat.quaternion = orientation
        pub.publish(quat)
        rospy.loginfo(rospy.get_name()+': Quaternion Published.')

    rospy.init_node('ar_to_quat_sofiehdfformat', anonymous=True)
    rospy.Subscriber("ar_pose_marker", ARMarker, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
