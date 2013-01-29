#!/usr/bin/env python
#
# Caution this file only imports the recorded values into the HDF file after the
#experiment is performed. The importARData function must thus be called.
# This is a hack because the PYTables does not allow concurrent writing to the file,
#thus a csv is created and then must be imported at a later stage. Needs refractoring.
import sys
import roslib; roslib.load_manifest('sofiehdfformat_rosdriver')
import rospy
import os.path
from ar_pose.msg import ARMarker

from sofiehdfformat.core.SofieCsvPyTableAccess import SofieCsvPyTableAccess
from sofiehdfformat.core.SofieCsvParser import parse_sample_interpret as csv_sample_interpret
from sofiehdfformat.core.SofieCsvAccess import SofieCsvAccess
from sofiehdfformat.core.SofieCsvFile import CsvFile
import signal
from sofiehdfformat_rosdriver.fileUtils import tableStructure
csvWriter = None;

def getFileInfo():
    '''
    Get the File info from the ros parameter server.
    '''
    csvfilename = None
    while csvfilename == None:
        try:
            csvfilename = rospy.get_param('/sofie/csvfilename')
        except KeyError:
            pass
    return csvfilename

def sofiewritercallback(data):
    '''
    Implements a simple writer to SOFIEHDFFORMAT file.
    '''
    rospy.logdebug(rospy.get_name() + ": Received Quaternion")
    rospy.logdebug(data)
    #The Quatertion must be in the format [w x y z] (or [w i j k]) for MATLAB3DSpace.
#    csvWriter.write(
#        {
#        'id':data.header.seq,
#        'confidence':data.confidence,
#        'x':data.pose.pose.position.x,
#        'y':data.pose.pose.position.y,
#        'z':data.pose.pose.position.z,
#        'quat1':data.pose.pose.orientation.w,
#        'quat2':data.pose.pose.orientation.x,
#        'quat3':data.pose.pose.orientation.y,
#        'quat4':data.pose.pose.orientation.z,
#        'timestamp':data.header.stamp.to_time()})
    csvWriter.write(
        (data.header.seq,
        data.confidence,
        data.pose.pose.position.x,
        data.pose.pose.position.y,
        data.pose.pose.position.z,
        data.pose.pose.orientation.w,
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.header.stamp.to_time()))

if __name__ == '__main__':
    csvfilename = getFileInfo()
    rospy.loginfo('Logging to file: {0}'.format(csvfilename))
    csvWriter = SofieCsvAccess(csvfilename, tableStructure)
    rospy.init_node('sofiehdfformatwriter', anonymous=True)
    rospy.Subscriber("ar_pose_marker", ARMarker, sofiewritercallback)
    rospy.spin()    