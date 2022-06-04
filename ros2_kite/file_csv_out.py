#!/usr/bin/env python

import os
import csv


class CSVDataWrite:
    """ For logging CSV files to disk for further analysis
    """

    def __init__(self):
        self.file_ref = None
        self.csvwriter = None

    def open_output(self, file_name='testoutput.csv', path='', reset_file=True):
        """ Opens a file for CSV data ouptut.

            If path is not specified (an empty string is given as
            the path) then the file will be opened in the current
            execution directory.

            If the reset_file parameter is False then file will be
            opened in append mode. If True then file will be opened
            in write mode and any existing data will be deleted if
            the file already exists.
        """

        # create the fully qualified path name
        file_path = os.path.join(path, file_name)
        fmode = "w" if reset_file else "a"
        try:
            self.file_ref = open(file_path, fmode)
            self.csvwriter = csv.writer(self.file_ref)
        except Exception as e:
            print("%s" % str(e))
        return

    def close_output(self):
        self.file_ref.close()
        return

    def write_data(self, datavals):
        self.csvwriter.writerow(datavals)
        return


def test_file():
    # this should create an output file
    out = CSVDataWrite()
    out.open_output()
    myheaders = ('One', 'Two', 'Three', 'Four')
    out.write_data(myheaders)
    for x in range(10):
        mydata = (1, 2, 3, 4)
        out.write_data(mydata)
    out.close_output()
    print('File output completed')


if __name__ == '__main__':
    test_file()
