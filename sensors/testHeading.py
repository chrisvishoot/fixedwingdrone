from heading import *
import time
A = (0.0, 0.0)
B = (-13.0, 0)
C = (-23.0, 41)
goal = (-34.0, 41.0) #goal point
currentHeading = 180
while (A[0] > -13):
    error = getTargetHeadingOffset(A, goal, currentHeading)
    A = (A[0] - 1, A[1])
    print "Error ", error
    print "Location ", A
    time.sleep(0.5)
currentHeading = 104.405003855

while (A[0] >= -23 and A[1] <= 41):
    error = getTargetHeadingOffset(A, goal, currentHeading)
    A = (A[0]- 0.25, A[1] + 1)
    print "Error ", error
    print "Location ", A
    time.sleep(0.5)

currentHeading = 180


while(A[0] >= -34):
    error = getTargetHeadingOffset(A, goal, currentHeading)
    A = (A[0]- 1, A[1])
    print "Error ", error
    print "Location ", A
    time.sleep(0.5)

print getTargetHeading(B,C)
