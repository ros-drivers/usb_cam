from sofiehdfformat.core.SofieFileUtils import importdata,exportFile
from sofiehdfformat.core.config import getBagSubDirectory,getDefaultBagFile,getDefaultBagFileName
import sys
import os
import logging
TMPCSVFILENAME='ar-csv.csv'
tableStructure = ['id', 'confidence', 'x', 'y', 'z', 'quat1', 'quat2', 'quat3', 'quat4', 'timestamp']
def importARData(csvFileName,hdfFilename,runName):
    print 'IMPORTING AR DATA'
    try:
        if os.path.isfile(csvFileName) and os.path.getsize(csvFileName) > 0:
            importdata(csvFileName,
                hdfFilename,
                runName,
                'description',
                True,
                False)
        print 'FINISHED PARSING AR INTO SOFIEHDFFILE.'
    except:
        print "Unexpected error:", sys.exc_info()[0]
        raise
    return True

def importBagData(usbCamBagFilename,hdfFilename,runName):
    importdata(usbCamBagFilename,
                hdfFilename,
                runName,
                'description',
                True,
                False)
def exportBagData(filename,runName):
    logging.debug(runName)
    totalRun =  getDefaultBagFile(getBagSubDirectory(runName))
    logging.debug(totalRun)
    exportFile(filename,totalRun,filename+'_'+getDefaultBagFileName())
    