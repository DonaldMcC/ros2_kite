from datetime import datetime
from file_csv_out import CSVDataWrite
from cvwriter import initwriter, writeframe


def writelogheader(config, kite, base, control):
    # THis should initialise the logging
    if config.logging:
        config.csvwriter = CSVDataWrite()
        config.csvwriter.open_output()
        config.csvwriter.write_data(('Seq No',) + kite.getlogheaders() + base.getlogheaders()
                                    + control.getlogheaders() + config.getlogheaders() + ('DateTime',))
    return


def writepictheader(config, height, width, fps):
    if config.logging and config.writer is None:
        # h, w = frame.shape[:2]
        # height, width = 480, 640 - removed should now be set above
        # height, width, channels = frame.shape
        config.writer = initwriter("record.avi", height, width, fps)


def writelogs(config, kite, base, control, frame, height, width, counter):
    if config.logging:  # not saving this either as it errors on other screen
        writeframe(config.writer, frame, height, width)
        #mydata = config.getlogdata()
        config.csvwriter.write_data(counter + kite.getlogdata() + base.getlogdata() +
                                    control.getlogdata() + config.getlogdata() + (datetime.now().strftime("%m/%d/%Y, %H:%M:%S.%f"),) )

def closelogs(config):
    config.csvwriter.close_output()
