# -*- coding: utf-8 -*-
# Calculation of direction between two geographical points
#
# To determine the direction from the starting point between two points on the earth, use the following formula: 
#
# Δφ = ln( tan( latB / 2 + π / 4 ) / tan( latA / 2 + π / 4) ) 
# Δlon = abs( lonA - lonB ) 
# bearing :  θ = atan2( Δlon ,  Δφ ) 
#
# Note: 1) ln = natural log      2) if Δlon > 180°  then   Δlon = Δlon (mod 180).

import math

#     #lat, lon
# A = (0.0, 0.0)
# B = (34.0 * math.pi / 180, -41.0 * math.pi / 180)


# Usage:
# Arguments are tuples that contain latitude and longitude, in degrees. 
# e.x. currentLocation = (0.0, 0.0) and targetLocation = (34.0, -41.0)
#      getTargetHeading(currentLocation, targetLocation) will return 318 degrees
def getTargetHeading(currentLocation, targetLocation):
    A = (currentLocation[0] * math.pi / 180, currentLocation[1] * math.pi / 180)
    B = (targetLocation[0] * math.pi / 180, targetLocation[1] * math.pi / 180)
    
    delta_phi = math.log(math.tan(B[0] / 2 + math.pi / 4) / math.tan(A[0] / 2 + math.pi / 4))
    delta_lon = math.fabs(A[1] - B[1])

    # If quadrant 2, add 3 * pi / 2 radians (270 degrees) offset
    # If quadrant 3, add pi / 2 radians (90 degrees) offset
    offset = 0
    if B[0] - A[0] < 0 and B[1] - A[1] < 0:
        offset = math.pi / 2
    elif B[0] - A[0] >= 0 and B[1] - A[1] < 0:
        offset = 3 * math.pi / 2

    theta = math.atan2(delta_lon, delta_phi) + offset
    theta = theta * 180 / math.pi

    return theta

# Usage:
# Locations are tuples that contain latitude and longitude, in degrees.
# currentHeading is also in degrees.
# A position return means your targetHeading is to the right (clockwise) of your currentHeading.
# A negative return means your targetHeading is to the left (counter-clockwise) of your current heading.
def getTargetHeadingOffset(currentLocation, targetLocation, currentHeading):
    target = getTargetHeading(currentLocation, targetLocation)
    offset = target - currentHeading
    
    if offset > 180:
        offset = offset - 360
        
    return offset
