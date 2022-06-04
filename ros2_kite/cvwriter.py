# initialize the FourCC, video writer, dimensions of the frame, and
# zeros array

from __future__ import print_function
import numpy as np
import cv2

fourcc = cv2.VideoWriter_fourcc("M", "J", "P", "G")
writer = None
(h, w) = (None, None)
zeros = None


# check if the writer is None
def initwriter(output, h, w, fps):
    # store the image dimensions, initialzie the video writer,
    # and construct the zeros array
    # (h, w) = frame.shape[:2]
    writer = cv2.VideoWriter(output, fourcc, fps, (w, h), True)
    return writer


def writeframe(writer, frame, h, w):
    # grab the frame from the video stream and resize it to have a
    # maximum width of 300 pixels
    # frame = imutils.resize(frame, width=300)
    output = np.zeros((h, w, 3), dtype="uint8")
    output[0:h, 0:w] = frame
    # write the output frame to file
    writer.write(output)
