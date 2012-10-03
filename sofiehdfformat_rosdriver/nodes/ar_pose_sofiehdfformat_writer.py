#!/usr/bin/env python
import sys
import roslib; roslib.load_manifest('sofiehdfformat_rosdriver')
import rospy
from ar_pose.msg import ARMarker

from sofiehdfformat.core.SofieCsvPyTableAccess import SofieCsvPyTableAccess
from sofiehdfformat.core.SofieCsvParser import parse_sample_interpret as csv_sample_interpret
from sofiehdfformat.core.SofieCsvAccess import SofieCsvAccess
from sofiehdfformat.core.SofieCsvFile import CsvFile
import signal

TMPCSVFILE='ar-csv-tm.csv'
csvWriter = None;
tableStructure = ['id', 'confidence', 'x', 'y', 'z', 'quat1', 'quat2', 'quat3', 'quat4', 'timestamp']

def getFileInfo():
    '''
    Get the File info from the ros parameter server.
    '''
    filename = None
    runName = None
    while filename == None or runName == None:
        try:
            filename = rospy.get_param('/sofie/filename')
            runName = rospy.get_param('/sofie/runname')
        except KeyError:
            pass
    return [filename, runName]

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
    
def exitCleanly(signum, frame):
    rospy.loginfo(rospy.get_name() + ": Exiting and closing files")
    try:
        theReader = CsvFile(TMPCSVFILE)
        csvWriter = SofieCsvPyTableAccess(filename, runName, tableStructure)
        for row in theReader:
            logging.debug("Read New Row")
            parsed = csv_sample_interpret(row)
            csvWriter.write(parsed)
        csvWriter.close();
        theReader.close();
    except:
        print "Unexpected error:", sys.exc_info()[0]
        raise

if __name__ == '__main__':
    argv = rospy.myargv(argv=sys.argv)
    print 'Waiting for filename and runName:' + str(len(argv))
    if len(argv) == 3:
        print 'Getting names from commandline.'
        filename = argv[1]
        runName = argv[2]
        rospy.set_param('/sofie/filename', filename)
        rospy.set_param('/sofie/runName', runName)

    [filename, runName] = getFileInfo()
    runName = '/'+runName.strip('/')+'/';
    rospy.loginfo('Logging to file: ' + filename + ' runName:' + runName)
    csvWriter = SofieCsvAccess(TMPCSVFILE, tableStructure)
    rospy.init_node('sofiehdfformatwriter', anonymous=True)
    rospy.Subscriber("ar_pose_marker", ARMarker, sofiewritercallback)
    signal = signal.signal(signal.SIGINT, exitCleanly)
    rospy.spin()
