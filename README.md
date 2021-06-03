
# Inerserting some information and importing modules such as keypressmodule in order to later down the line do more complex functions in further steps
#Section 1

from djitellopy import Tello
import KeyPressModule as kp
import cv2
import numpy as np
import time
import math

### END CODE SECTION 1 ###

#Section 2
# Assigning values to the speed varibles that can be used later in the code 

############ PARAMETERS ############
fSpeed = 56  # Forward/Backward cm/sec
aSpeed = 26  # Angular Velocity, Degrees/sec
interval = 0.25

dInterval = fSpeed * interval  # Distance Interval
aInterval = aSpeed * interval  # Angular Interval
#########################################


### END CODE SECTION 2 ###

x, y = 500, 500
a = 0
yaw = 0

kp.init()
tello = Tello()
# tello.connect()
# print(tello.get_battery())

points = []

# global img
# tello.streamon()


# Section 3
# By using kp module along with our intervals we got before, we are able to map out how the drone can move when reacting to a key press. We also define the varibles lr,fb,ud,yv and give them values

def getKeyboardInput():
    lr, fb, ud, yv = 0, 0, 0, 0
    speed = 50
    global x, y, yaw, a
    d = 0

    if kp.getKey("LEFT"):
        lr = -speed
        d = dInterval
        a = 180

    elif kp.getKey("RIGHT"):
        lr = speed
        d = -dInterval
        a = -180

    if kp.getKey("UP"):
        fb = speed
        d = dInterval
        a = 270

    elif kp.getKey("DOWN"):
        fb = -speed
        d = -dInterval
        a = -90

    if kp.getKey("w"):
        ud = speed
    elif kp.getKey("s"):
        ud = -speed

    if kp.getKey("a"):
        yv = -speed
        yaw += aInterval

    elif kp.getKey("d"):
        yv = speed
        yaw -= aInterval

    if kp.getKey("q"):
        tello.land()
        time.sleep(3)
    if kp.getKey("e"): tello.takeoff()
    
### END CODE SECTION 3 ###    


 #   if kp.getKey("z"):
 #       cv2.imwrite(f'Resources/Images/{time.time()}.jpg', img)
 #       time.sleep(0.3)
 
    time.sleep(interval)
    a += yaw
    x += int(d * math.cos(math.radians(a)))
    y += int(d * math.sin(math.radians(a)))

    return [lr, fb, ud, yv, x, y]


#Section 4
# drawing a circular point on the map for the drone at a location, giving it the size, color, and filled in.


def drawPoints(img, points):
    for point in points:
        cv2.circle(img, point, 5, (0, 0, 255), cv2.FILLED)

### END CODE SECTION 4 ###    


#Section 5
# checks for values and updates whenever one is entered. Also connects the points together to form lines, creating the actual map.

while True:
    vals = getKeyboardInput()
    tello.send_rc_control(vals[0], vals[1], vals[2], vals[3])
    img = np.zeros((1000, 1000, 3), np.uint8)
    points.append((vals[4], vals[5]))
    drawPoints(img, points)
    cv2.imshow("Output", img)
    cv2.waitKey(1)
    
### END CODE SECTION 5 ###    


### Ignore the following code, it's related to the Surveillance program and not used in the MAPPING Program ###

# img = tello.get_frame_read().frame
# img = cv2.resize(img, (360, 240))
# cv2.imshow("image", img)
# cv2.waitKey(1)
