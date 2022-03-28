"""
Code written based on servoj example from: https://github.com/davizinho5/RTDE_control_example
"""
import sys
sys.path.append('')
import logging
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import numpy as np
import time
from matplotlib import pyplot as plt
from Scripts.trajectory import asym_trajectory, log_traj, plot_traj ,inital_parameters_traj
from Scripts.Kinematic import inverse_kinematic
# _____________functions_____________
def setp_to_list(setp):
    temp = []
    for i in range(0, 6):
        temp.append(setp.__dict__["input_double_register_%i" % i])
    return temp

#HElllo
def list_to_setp(setp, list):
    for i in range(0, 6):
        setp.__dict__["input_double_register_%i" % i] = list[i]
    return setp


# -------------------cos angles-----------------------------
w = np.linspace(0, 2, 1000)
# ------------- robot communication stuff -----------------
ROBOT_HOST = '169.254.182.10'
ROBOT_PORT = 30004
config_filename = r'D:\OneDrive - NTNU\Programmering\Python\Prosjekt\Bachelor\Source\Bachelor\Scripts\control_loop_configuration_simple.xml'  # specify xml file for data synchronization

logging.getLogger().setLevel(logging.INFO)

conf = rtde_config.ConfigFile(config_filename)
state_names, state_types = conf.get_recipe('state')  # Define recipe for access to robot output ex. joints,tcp etc.
setp_names, setp_types = conf.get_recipe('setp')  # Define recipe for access to robot input
watchdog_names, watchdog_types = conf.get_recipe('watchdog')

# -------------------- Establish connection--------------------
con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
connection_state = con.connect()

while not con.is_connected():
    time.sleep(0.5)
    con.connect()

print("---------------Successfully connected to the robot-------------\n")
# get controller version
con.get_controller_version()

# ------------------- setup recipes ----------------------------
FREQUENCY = 500  # send data in 500 Hz instead of default 125Hz
con.send_output_setup(state_names, state_types, FREQUENCY)
setp = con.send_input_setup(setp_names,
                            setp_types)  # Configure an input package that the external application will send to the robot controller
watchdog = con.send_input_setup(watchdog_names, watchdog_types)

# ---------------------registers:-------------------------------
setp.input_double_register_0 = 0
setp.input_double_register_1 = 0
setp.input_double_register_2 = 0
setp.input_double_register_3 = 0
setp.input_double_register_4 = 0
setp.input_double_register_5 = 0

setp.input_bit_registers0_to_31 = 0

watchdog.input_int_register_0 = 0

# start data synchronization
if not con.send_start():
    sys.exit()

state = con.receive()
tcp1 = state.actual_TCP_pose
con.send(watchdog)
#   ------------  mode = 1 (Connection) -----------
while True:
    # print('Boolean 1 is False, please click CONTINUE on the Polyscope')
    state = con.receive()
    print(state.actual_TCP_speed[0])
    
    
    #print(f"runtime state is {state.runtime_state}")
