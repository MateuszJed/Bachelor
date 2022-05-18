import asyncio
import sys
# sys.path.insert(0, "..")
import logging
from asyncua import Client, Node, ua
from controller import Robot, Motor, PositionSensor, GPS

# create the Robot instance.
robot = Robot()

timestep = int(robot.getBasicTimeStep())

################### Client ###################
logging.basicConfig(level=logging.INFO)
_logger = logging.getLogger('asyncua')
##############################################


base = robot.getDevice("Base_motor")
shoulder = robot.getDevice("Shoulder_motor")
elbow = robot.getDevice("Elbow_motor")

speed =1

#base.setPosition(float("inf"))
base.setVelocity(speed)

shoulder.setPosition(float("inf"))
shoulder.setVelocity(speed)

elbow.setPosition(float("inf"))
elbow.setVelocity(speed)

gps = GPS("gps")
gps.enable(20)



async def main():
    url = 'opc.tcp://169.254.182.5:8000'
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
            
            gps_values = gps.getValues()
            
            
            print(base_pos,shoulder_pos,elbow_pos)
            #print(gps_values)
            base.setPosition(base_pos)
            shoulder.setPosition(shoulder_pos)
            elbow.setPosition(elbow_pos)
    
if __name__ == '__main__':
    asyncio.run(main())
    