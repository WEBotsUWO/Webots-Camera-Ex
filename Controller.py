"""my_controller controller."""


import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import random

# Import Webots robot libraries
from controller import Robot, Motor, DistanceSensor, Camera, Lidar#, LidarPoint
os.environ['WEBOTS_ROBOT_NAME'] = "TS1" # Create robot name environment variable

# create the Robot instance.
robot = Robot()

# get the time step of the current world
#timestep = int(robot.getBasicTimeStep())
timestep = 1000

# set downsampling factor for camera and lidar images
downsamplefac = 8

# Start motors and put robot into right position:
fixedmotor0 = robot.getMotor('torso_lift_joint')
fixedmotor0.setPosition(0)
fixedmotor1 = robot.getMotor('arm_1_joint')
fixedmotor1.setPosition(1.57)
fixedmotor2 = robot.getMotor('arm_2_joint')
fixedmotor2.setPosition(-1.5)
fixedmotor3 = robot.getMotor('arm_3_joint')
fixedmotor3.setPosition(-3.14)
fixedmotor4 = robot.getMotor('arm_4_joint')
fixedmotor4.setPosition(1.57)
fixedmotor6 = robot.getMotor('arm_6_joint')
fixedmotor6.setPosition(0)
fixedmotor7 = robot.getMotor('head_1_joint')
fixedmotor7.setPosition(0)
fixedmotor8 = robot.getMotor('head_2_joint')
fixedmotor8.setPosition(0)


# Start movement motors:
wheel_left = robot.getMotor('wheel_left_joint')
wheel_left.setVelocity(0)
wheel_right = robot.getMotor('wheel_right_joint')
wheel_right.setVelocity(0)
# Disable motor PID control mode.
wheel_left.setPosition(float('inf'))
wheel_right.setPosition(float('inf'))

# Initialize movement actions:
def move_forward():
    wheel_left.setVelocity(5)
    wheel_right.setVelocity(5)
    
def move_backward():
    wheel_left.setVelocity(-5)
    wheel_right.setVelocity(-5)
    
def move_left():
    wheel_left.setVelocity(-5)
    wheel_right.setVelocity(5)
    
def move_right():
    wheel_left.setVelocity(5)
    wheel_right.setVelocity(-5)

actions = ['forw', 'backw', 'left', 'right']

# Start camera
cam = robot.getCamera('camera left camera')
cam.enable(timestep)
camwid = cam.getWidth()
camhei = cam.getHeight()
imgwid = int(camwid/downsamplefac)
imghei = int(camhei/downsamplefac)

# Start front lidar sensor
lid = robot.getLidar('Hokuyo')
lid.enable(timestep)
lid.enablePointCloud()
lidimg = lid.getPointCloud()
numlid = len(lidimg)


# # Start back distance sensors
# bds1 = robot.getDistanceSensor('base_sonar_01_link')
# bds1.enable(timestep)
# bds2 = robot.getDistanceSensor('base_sonar_02_link')
# bds2.enable(timestep)
# bds3 = robot.getDistanceSensor('base_sonar_03_link')
# bds3.enable(timestep)

# Start arm position sensors
as1 = robot.getPositionSensor('arm_1_joint_sensor')
as1.enable(timestep)
as2 = robot.getPositionSensor('arm_2_joint_sensor')
as2.enable(timestep)
as3 = robot.getPositionSensor('arm_3_joint_sensor')
as3.enable(timestep)
as4 = robot.getPositionSensor('arm_4_joint_sensor')
as4.enable(timestep)
as5 = robot.getPositionSensor('arm_5_joint_sensor')
as5.enable(timestep)
as6 = robot.getPositionSensor('arm_6_joint_sensor')
as6.enable(timestep)
as7 = robot.getPositionSensor('arm_7_joint_sensor')
as7.enable(timestep)
fs1 = robot.getPositionSensor('gripper_left_finger_joint_sensor')
fs1.enable(timestep)
fs2 = robot.getPositionSensor('gripper_right_finger_joint_sensor')
fs2.enable(timestep)

j = 0
while robot.step(timestep) != -1 and j < 30:
    j += 1
    
    # Stop robot
    wheel_left.setVelocity(0)
    wheel_right.setVelocity(0)
    fixedmotor7.setPosition(0)
    fixedmotor8.setPosition(0)
    
    # Get grayscale camera image:
    print(timestep)
    #img = cam.getImageArray()              # does not seem to work
    #gray = img[:][:][3]
    gray = np.empty(shape=(imghei,imgwid),dtype='float') 
    img = cam.getImage()
    xc = 0
    for x in range(0, camwid, downsamplefac):
        yc = 0
        for y in range(0, camhei, downsamplefac):
            #print(str(x)+","+str(y))
            gray[yc,xc] = Camera.imageGetGray(img, camwid, x, y)
            yc += 1
        xc += 1
            
    # # Get lidar points and convert them to distance image:
    liddist = np.empty(shape=(numlid),dtype='float') 
    lidimg = lid.getPointCloud()
    for poi in range(0, numlid):
        liddist[poi] = (lidimg[poi].x**2 + lidimg[poi].y**2 + lidimg[poi].z**2)**(1/2)
    del lidimg
    liddist = np.random.rand(numlid)              
            
    # # Get back distance sensor data:
    # backdist = np.empty(shape=(1,3),dtype='float')
    # backdist[0,0] = bds1.getValue()
    # backdist[0,1] = bds2.getValue()
    # backdist[0,2] = bds3.getValue()
    
    # Get arm sensor data:
    arm = np.empty(shape=(1,9),dtype='float')
    arm[0,0] = as1.getValue()
    arm[0,1] = as2.getValue()
    arm[0,2] = as3.getValue()
    arm[0,3] = as4.getValue()
    arm[0,4] = as5.getValue()
    arm[0,5] = as6.getValue()
    arm[0,6] = as7.getValue()
    arm[0,7] = fs1.getValue()
    arm[0,8] = fs2.getValue()
    
    
    action = actions[random.randrange(len(actions))]

    if action == "forw":
        move_forward()
    elif action == "backw":
        move_backward()
    elif action == "left":
        move_left()
    elif action == "right":
        move_right()
    pass

# Enter here exit cleanup code.




#plt.imshow(gray,aspect="auto",cmap=plt.cm.gray)
#plt.imshow(liddist,aspect="auto",cmap=plt.cm.gray)
