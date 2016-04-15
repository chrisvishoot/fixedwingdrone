import math

#A radius of 10 meters, this is subject to change.
IDEAL_RADIUS = 10
IDEAL_THETA = 20 #just some arbritrary angle
#prevErrorZ = 0 #initialize to zero

#Assuming that I get the x, y, and z cordinates from the GPS cordinates
#This method will return the theta used for polar cordinates
#This will calculate the angle theta relative from the "end goal"
def getTheta(origX, origY, currX, currY):
    x = currX - origX
    y = currY - origY
    rads = math.atan(y/x)
    if(y < 0 and x < 0):
        rads = rads + math.pi
    if(y > 0 and x < 0):
        rads = rads + math.pi
    return (rads * 180) / math.pi
#In order to calculate the phi, I will need the z cordinate and the distance / radius "rho" in the
# 3rd dimension
#arcosine has a domain 0 to pi, and this will be calcuating the phi, or the angle of how
#far above or below the drone is for the target end goal
def getPhi(origZ, currZ, rho):
    z = currZ - origZ
    phi = math.acos(z / rho)
    return phi * 180 / math.pi


#Gets the current radius relative to the plane
#The origin will be the goal cordinates, and the end points will refer to the
#current x y location of the plane.
def getRho(origX, origY, origZ, currX, currY, currZ):
    xdiff = currX - origX
    ydiff = currY - origY
    zdiff = currZ - origZ
    xSquared = xdiff * xdiff
    ySquared = ydiff * ydiff
    zSquared = zdiff * zdiff
    return math.sqrt(xSquared + ySquared + zSquared)

def getRadius(origX, origY, currX, currY):
    xdiff = currX - origX
    ydiff = currY - origY
    xSquared = xdiff * xdiff
    ySquared = ydiff * ydiff
    return math.sqrt(xSquared + ySquared + zSquared)



#gets the error angle of the plane
#In order to do this right, I will need the angle direction
#I believe we already calculated this last time with Such Kamal.
#Bring this up during the team meeting.
def errorXYPlane(currRadius, refRadius, currAngle, direction):
    numerator = currRadius - refRadius * math.cos(currAngle)
    denominator = currRadius * math.sin(currAngle)
    theta = (math.atan(numerator / denominator) * 180) / math.pi
    phi = (direction - 360) % 360
    totalError = 90 + theta - phi
    return totalError
#Gets the error for the height z.
def errorZPlane(currZ, targetZ):
    return targetZ - currZ

#Control input for the 2D plane
def controlInputXY(Kp, errorXY):
    return Kp * errorXY

#Control in the Z axis
def controlInputZ(Kp, Ki, prevErrorZ, currErrorZ):
    #discrete summation to represent an integral
    integral = currErrorZ + prevErrorZ
    control = Kp * currError + Ki * integral
    return control

def correctZError(error, currZ):
    if(error < 0):
        #Increment z as if its increasing the drone's height
        currZ = currZ - 1
    elif(error > 0):
        #drone going down
        currZ = currZ + 1
    return currZ
