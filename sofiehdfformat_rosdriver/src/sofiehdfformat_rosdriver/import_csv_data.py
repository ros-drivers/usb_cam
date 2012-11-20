from sofiehdfformat.core.SofieFileUtils import importdata
import sys
import os
TMPCSVFILE='/tmp/ar-csv-tm.csv'
tableStructure = ['id', 'confidence', 'x', 'y', 'z', 'quat1', 'quat2', 'quat3', 'quat4', 'timestamp']
def importARData(filename, runName):
    print 'IMPORTING AR DATA'
    try:
        if os.path.isfile(TMPCSVFILE):
            importdata(TMPCSVFILE,
                filename,
                runName,
                'description',
                True,
                False)
        print 'FINISHED PARSING AR INTO SOFIEHDFFILE.'
    except:
        print "Unexpected error:", sys.exc_info()[0]
        raise
    return True

def importBagData(filename, usbCamBagFilename,runName):
    importdata(usbCamBagFilename,
                filename,
                runName,
                'description',
                True,
                False)