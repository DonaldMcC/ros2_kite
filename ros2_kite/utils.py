# import the necessary packages
# from http://www.pyimagesearch.com/2016/03/21/ordering-coordinates-clockwise-with-python-and-opencv/
from scipy.spatial import distance as dist
import numpy as np


def order_points(pts):
    # sort the points based on their x-coordinates
    """
    >>> order_points(np.array([(4,4),(2,4),(2,2),(4,2)]))
    array([[2, 2],
           [4, 2],
           [4, 4],
           [2, 4]], dtype=uint8)
    """

    xSorted = pts[np.argsort(pts[:, 0]), :]

    # grab the left-most and right-most points from the sorted
    # x-roodinate points
    leftMost = xSorted[:2, :]
    rightMost = xSorted[2:, :]

    # now, sort the left-most coordinates according to their
    # y-coordinates so we can grab the top-left and bottom-left
    # points, respectively
    leftMost = leftMost[np.argsort(leftMost[:, 1]), :]
    (tl, bl) = leftMost

    # now that we have the top-left coordinate, use it as an
    # anchor to calculate the Euclidean distance between the
    # top-left and right-most points; by the Pythagorean
    # theorem, the point with the largest distance will be
    # our bottom-right point
    D = dist.cdist(tl[np.newaxis], rightMost, "euclidean")[0]
    (br, tr) = rightMost[np.argsort(D)[::-1], :]

    # return the coordinates in top-left, top-right,
    # bottom-right, and bottom-left order
    return np.array([tl, tr, br, bl], dtype="int16")


def _test():
    import doctest
    doctest.testmod()


if __name__ == '__main__':
    'Can run with -v option if you want to confirm tests were run'
    _test()
