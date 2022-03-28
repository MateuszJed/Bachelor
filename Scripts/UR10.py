import sys
sys.path.append('')
import logging
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config

def initial_communiation(Robot_IP,Robot_Port,FREQUENCY):
    ROBOT_HOST = Robot_IP
    ROBOT_PORT = Robot_Port
    config_filename = r'D:\OneDrive - NTNU\Programmering\Python\Prosjekt\Bachelor\Source\Bachelor\Scripts\control_loop_configuration_simple.xml'

    logging.getLogger().setLevel(logging.INFO)

    conf = rtde_config.ConfigFile(config_filename)
    state_names, state_types = conf.get_recipe('state')  # Define recipe for access to robot output ex. joints,tcp etc.
    setp_names, setp_types = conf.get_recipe('setp')  # Define recipe for access to robot input
    watchdog_names, watchdog_types = conf.get_recipe('watchdog')
    con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)


    connection_state = con.connect()

    while not con.is_connected():
        time.sleep(0.5)
        con.connect()
    con.get_controller_version()

    # ------------------- setup recipes ----------------------------
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
    #   ------------  mode = 1 (Connection) -----------
    while True:
        state = con.receive()
        con.send(watchdog)
        print(f"Runtime state is {state.runtime_state}")
        if state.output_bit_registers0_to_31 == True:
            print('Boolean 1 is True, Robot Program can proceed to mode 1\n')
            break
    return setp,con,watchdog,state.actual_TCP_pose[:3]
# # initial_communiation('169.254.182.10', 30004)

