#!/usr/bin/env python
import sys
import roslib; roslib.load_manifest('sofiehdfformat_rosdriver')
import rospy
from geometry_msgs.msg import QuaternionStamped
from sofiehdfformat.core.SofieCsvPyTableAccess import SofieCsvPyTableAccess

tableStructure=['quat1','quat2','quat3','quat4','Timestamp']
def getFileInfo():
    filename=None
    runName=None
    while filename==None or runName == None:
        try:
            filename = rospy.get_param('/sofie/filename')
            runName = rospy.get_param('/sofie/runname')
        except KeyError:
            print 'Waiting for filename to be set.'
    return [filename,runName]

def callback(data):
        try:
            shouldWeLog = rospy.get_param('/sofie/shouldWeLog')
            if shouldWeLog==0:
                return
        except KeyError:
            return
        rospy.loginfo(rospy.get_name()+": Received Quaternion")
        rospy.logdebug(data)
        csvWriter.write(
            {'quat1':data.quaternion.x,
            'quat2':data.quaternion.y,
            'quat3':data.quaternion.z,
            'quat4':data.quaternion.w,
            'Timestamp':data.header.stamp.to_time()})

if __name__ == '__main__':
    argv = rospy.myargv(argv=sys.argv)
    if len(argv) == 3:
        filename = argv[1]
        runName = argv[2]
        rospy.set_param('/sofie/filename',filename)
        rospy.set_param('/sofie/runName',runName)
    else:
        [filename,runName] = getFileInfo()
    rospy.loginfo('Logging to file: '+filename+' runName:'+runName)
    csvWriter = SofieCsvPyTableAccess(filename,runName,tableStructure)
    rospy.init_node('sofiehdfformatwriter', anonymous=True)
    rospy.Subscriber("quat_sofiehdfformat", QuaternionStamped, callback)
    rospy.spin()
