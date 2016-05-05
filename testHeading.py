from heading import *
#latitude and
A = (0.0, 0.0) #initial point
B = (-34.0, 41.0) #goal point
currentHeading = 90
while (A[1] < 90):
    error = getTargetHeadingOffset(A, B, currentHeading)
    A = (0.0, A[1] + 1)
    print "Error ", error
    print "Location ", A
