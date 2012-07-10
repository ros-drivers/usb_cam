#!/usr/bin/env python
import sys
import roslib; roslib.load_manifest('sofiehdfformat_rosdriver')
import rospy
from geometry_msgs.msg import QuaternionStamped
from sofiehdfformat.core.SofieCsvPyTableAccess import SofieCsvPyTableAccess

tableStructure=['quat1','quat2','quat3','quat4','timestamp']
def getFileInfo():
    filename=None
    runName=None
    while filename==None or runName == None:
        try:
            filename = rospy.get_param('/sofie/filename')
            runName = rospy.get_param('/sofie/runname')
        except KeyError:
            pass
    return [filename,runName]

def callback(data):
        rospy.loginfo(rospy.get_name()+": Received Quaternion")
        rospy.logdebug(data)
        csvWriter.write(
            {
            'quat1':data.quaternion.w,
            'quat2':data.quaternion.x,
            'quat3':data.quaternion.y,
            'quat4':data.quaternion.z,
            'timestamp':data.header.stamp.to_time()})

if __name__ == '__main__':
    argv = rospy.myargv(argv=sys.argv)
    print 'Waiting for filname and runName:'+str(len(argv))
    if len(argv) == 3:
        print 'Getting names from commandline.'
        filename = argv[1]
        runName = argv[2]
        rospy.set_param('/sofie/filename',filename)
        rospy.set_param('/sofie/runName',runName)

    [filename,runName] = getFileInfo()
    rospy.loginfo('Logging to file: '+filename+' runName:'+runName)
    csvWriter = SofieCsvPyTableAccess(filename,runName,tableStructure)
    rospy.init_node('sofiehdfformatwriter', anonymous=True)
    rospy.Subscriber("quat_sofiehdfformat", QuaternionStamped, callback)
    rospy.spin()
