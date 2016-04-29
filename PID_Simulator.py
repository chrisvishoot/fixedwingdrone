#Import my PID controller
from PID_Implementation import *

import random
import time
#Steps for correcting the drone!
# 1) Correct the Z axis
# 2) Correct the radius length and the theta on the xy plane simulataniously


#Useful methods
#def getDistance(origX, origY, origZ, currX, currY, currZ):
#
#
#I have this reversed, the origY will be the end goal, and the curr is the current
#Location of the drone!
#I will be doing everything relative to the goal
#def getTheta(origX, origY, currX, currY):
#
#def getPhi(origZ, currZ, rho):
#
#
#def errorZPlane(currZ, targetZ):
#def correctZError(error, currZ):
#Generates a random set of points
def getPoints():
    x = random.randrange(0,100)
    y = random.randrange(0,100)
    z = random.randrange(0,100)
    return (x,y,z)





#Main method
def simulation():
    currentLocation = (98, 48, 83)
    end = (35, 42, 35)
    currDist = getRho(end[0], end[1], end[2], currentLocation[0], currentLocation[1], currentLocation[2])
    theta = getTheta(end[0], end[1], currentLocation[0],currentLocation[1])
    Zphi = getPhi(end[2], currentLocation[2], currDist)

    endZ = end[2]
    #currZ = currentLocation[2]

    endY = end[1]
    currY = currentLocation[1]

    endX = end[0]
    currX = currentLocation[0]

    #Corrects the z plane error
    currZ = float(input())
    zError = errorZPlane(currZ, endZ)
    while(zError != 0):
        currZ = correctZError(zError, currZ)
        zError = errorZPlane(currZ, endZ)
        print("My current z location is " + str(currZ))
    	print("Z Error " + str(zError))
	time.sleep(0.15)
    #prints out the should be corrected z value, for this case it should be 35.
    print (currZ)





if __name__ == '__main__':
    simulation()
