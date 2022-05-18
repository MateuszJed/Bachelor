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
################### Client ###################
logging.basicConfig(level=logging.INFO)
_logger = logging.getLogger('asyncua')
##############################################

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

base = robot.getDevice("shoulder_pan_joint")
base.setPosition(pi/2)

shoulder = robot.getDevice("shoulder_lift_joint")
shoulder.setPosition(-pi/2)

elbow = robot.getDevice("elbow_joint")
elbow.setPosition(pi/2)

wrist_1_joint = robot.getDevice("wrist_1_joint")
wrist_1_joint.setPosition(-pi/2)

wrist_2_joint = robot.getDevice("wrist_2_joint")
wrist_2_joint.setPosition(pi)

async def main():
#url = 'opc.tcp://169.254.182.11:8000'
    url = 'opc.tcp://localhost:4840'
    async with Client(url=url) as client:
        while robot.step(timestep) != -1:
            #base.setVelocity(speed)
            #shoulder.setVelocity(speed)
            #elbow.setVelocity(speed)
            opc_base = client.get_node("ns=5;i=1")
            opc_shoulder = client.get_node("ns=6;i=1")
            opc_elbow = client.get_node("ns=7;i=1")
            
            base_pos = await opc_base.read_value()
            shoulder_pos = await opc_shoulder.read_value()
            elbow_pos = await opc_elbow.read_value()
            
            base.setPosition(base_pos)
            shoulder.setPosition(shoulder_pos)
            elbow.setPosition(elbow_pos)
    
if __name__ == '__main__':
    asyncio.run(main())
    

# Enter here exit cleanup code.
