import asyncio
import sys
# sys.path.insert(0, "..")
import logging
from asyncua import Client, Node, ua
from controller import Robot, Motor, PositionSensor, GPS
from math import pi
# create the Robot instance.
robot = Robot()

gps = GPS("gps")
gps.enable(20)

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

base = robot.getDevice("shoulder_pan_joint")
base.setPosition(-pi/2)

shoulder = robot.getDevice("shoulder_lift_joint")
shoulder.setPosition(-pi/2)

elbow = robot.getDevice("elbow_joint")
elbow.setPosition(pi/2)

wrist_1_joint = robot.getDevice("wrist_1_joint")
wrist_1_joint.setPosition(-pi/2)

wrist_2_joint = robot.getDevice("wrist_2_joint")
wrist_2_joint.setPosition(-pi/2)



while robot.step(timestep) != -1:

    base.setPosition(0)
    shoulder.setPosition(0)
    elbow.setPosition(0)

    #x = -703.9mm
    #y = 105.23mm
    #z = -363.7mm
    #base.setPosition(-43.58*(pi/180))# joint -0.7606144880191288
    #shoulder.setPosition(-84.22*(pi/180))# joint -1.4699162960296244
    #elbow.setPosition(108.41*(pi/180))# joint 1.8921114420870526
    pass
    
#1.7 -0.164 0.625
    

# Enter here exit cleanup code.
