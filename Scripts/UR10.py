import sys,time
sys.path.append('')
import logging
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config

def initial_communiation(Robot_IP,Robot_Port,FREQUENCY,testing):
    ROBOT_HOST = Robot_IP
    ROBOT_PORT = Robot_Port
    config_filename = r'C:\Users\mateusz.jedynak\OneDrive - NTNU\Programmering\Python\Prosjekt\Bachelor\Source\Bachelor\Scripts\control_loop_configuration_simple.xml'

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
    if testing == "pid":

        #Regulation with PID
        setp.input_double_register_0 = 1.588424152240039    #91.01°
        setp.input_double_register_1 = -1.240405499392370   #-71.07°
        setp.input_double_register_2 = 1.191187214486130    #68.25°
        setp.input_double_register_3 = -1.57                #-90°
        setp.input_double_register_4 = -3.14                #-180°
        setp.input_double_register_5 = 1.57                 #90
      
    if testing == "kinematic":
        setp.input_double_register_0 = 1.6690087429152163          
        setp.input_double_register_1 = -1.0156021332056562
        setp.input_double_register_2 =  1.6942015789157032
        setp.input_double_register_3 = -1.57                #-90°
        setp.input_double_register_4 = -3.14                #-180°
        setp.input_double_register_5 = 1.57                 #90

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
            con.send(setp)
            time.sleep(0.01)
            break
    return setp,con,watchdog,[0.0277,-0.8845,0.7411]
# # initial_communiation('169.254.182.10', 30004)

