#!/usr/bin/env python
import roslib; roslib.load_manifest('sofiehdfformat_rosdriver')
import rospy
from geometry_msgs.msg import QuaternionStamped
from sofiehdfformat.core.SofieCsvPyTableAccess import SofieCsvPyTableAccess



def listener():
    csvWriter = SofieCsvPyTableAccess('./test.h5','testrun',
        ['quat1','quat2','quat3','quat4','Timestamp'])
    def callback(data):
        rospy.loginfo(rospy.get_name()+"Received Quaternion ")
        rospy.loginfo(data)
        csvWriter.write(
            {'quat1':data.quaternion.x,
            'quat2':data.quaternion.y,
            'quat3':data.quaternion.z,
            'quat4':data.quaternion.w,
            'Timestamp':data.header.stamp.to_time()})
    rospy.init_node('sofiehdfformatwriter', anonymous=True)
    rospy.Subscriber("quat_sofiehdfformat", QuaternionStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
