from sofiehdfformat.core.SofieCsvPyTableAccess import SofieCsvPyTableAccess
from sofiehdfformat.core.SofieCsvParser import parse_sample_interpret as csv_sample_interpret
from sofiehdfformat.core.SofieCsvAccess import SofieCsvAccess
from sofiehdfformat.core.SofieCsvFile import CsvFile
import sys
TMPCSVFILE='/tmp/ar-csv-tm.csv'
tableStructure = ['id', 'confidence', 'x', 'y', 'z', 'quat1', 'quat2', 'quat3', 'quat4', 'timestamp']
def importARData(filename, runName):
    print '..importing the AR information'
    try:
        theReader = CsvFile(TMPCSVFILE)
        print '.'
        csvWriter = SofieCsvPyTableAccess(filename, runName, tableStructure)
        for row in theReader:
            parsed = csv_sample_interpret(row)
            csvWriter.write(parsed)
        csvWriter.close();
        theReader.close();
        print 'FINISHED PARSING AR INTO SOFIEHDFFILE.'
    except:
        print "Unexpected error:", sys.exc_info()[0]
        raise
    return True