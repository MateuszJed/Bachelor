from mimetypes import init
import cv2,math,time,keyboard,csv,cmath
import pyrealsense2 as rs
import numpy as np
from control.matlab import *
from Scripts.Kinematic import inverse_kinematic,forwad_kinematic,forwad_kinematic_v2
from Scripts.Camera import Inital_color,Object_3D_recontruction,Camera_top_to_qlobal_coords
from Scripts.miscellaneous import _map,setp_to_list,list_to_setp
from Scripts.UR10 import initial_communiation
from Scripts.trajectory import asym_trajectory,inital_parameters_traj

detected = False
show_cam = True
flip_cam = False

#Config IntelRealsens
lower_color, upper_color = Inital_color("yellowbox")

pipeline = rs.pipeline()
config = rs.config()
# config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.color, 848,480, rs.format.bgr8, 60)
pipeline.start(config)
#===============================End of functions=================================================================================
def Angle(UR_10_x,UR_10_y,object_x,object_y,l):
    return cmath.asin((object_x-UR_10_x)/l).real,cmath.asin((object_y-UR_10_y)/l).real
def find_lqr_control_input(UR_mass,payload_mass,payload_length,g,demp,b,pos,velocity,theta_pos,theta_velocity):
	# Using LQR to find control inputs
	
	# The A and B matrices are derived in the report
	A = np.matrix(  [
					[0,1,0,0],
					[0,b*demp/UR_mass,b*payload_mass/UR_mass,0],
					[0,0,0,1],
					[0,-b*demp/(UR_mass*payload_length),-b*(UR_mass+payload_mass)*g/(UR_mass*payload_length),0]
					])
	
	B = np.matrix(	[
					[0],
					[1/UR_mass],
					[0],
					[b*1/(payload_length*UR_mass)]
					])

	# The Q and R matrices are emperically tuned. It is described further in the report
	Q = np.matrix(	[
					[1,0,0,0],
					[0,1,0,0],
					[0,0,1,0],
					[0,0,0,1]
					])

	R = np.matrix([50])

	# The K matrix is calculated using the lqr function from the controls library 
	K, S, E = lqr(A, B, Q, R)
	np.matrix(K)

	x = np.matrix(	[
					[np.squeeze(np.asarray(pos))],
					[np.squeeze(np.asarray(velocity))],
					[np.squeeze(np.asarray(theta_pos))],
					[np.squeeze(np.asarray(theta_velocity))]
					])

	desired = np.matrix(	[
					[0],
					[0],
					[0],
					[0]
					])

	F = -(K*(x-desired))
	return np.squeeze(np.asarray(F))
def main():
    v_0_x,v_2_x,v_0_y,v_2_y,t_0,t_1,t_f,distance = 0,0,0,0,0,1.5,0.75,1.48
    prev_error_x, prev_error_y,reference_point_x,reference_point_y,prev_pos = 0,0,0,0,0
    eintegral_x,eintegral_y,prev_angle_x,prev_angle_y,prev_velocity_x,prev_velocity_y= 0,0,0,0,0,0
    pf,running = True,True
    start_time = time.time()
    # Client has a few methods to get proxy to UA nodes that should always be in address space such as Root or Objects
    setp,con,watchdog,Init_pose = initial_communiation('169.254.182.10', 30004,500)
    watchdog.input_int_register_0 = 2
    con.send(watchdog)  # sending mode == 2
    state = con.receive()
    # K = np.array([[316.2277660168385, 411.4651103132678, -65.82013559986018, -7.307886819799695]])
    UR_MASS = 1
    PAYLOAD_MASS = 0.5
    ROPE_LENGTH = 1.4
    DEMP_COFF = 0.0028
    GRAVITY = -9.81
    PENDULUM = -1
    # global_coordinates = [0,0,0,0]
    # angle = [7*math.pi/4,0,0,0]
    # angle = [math.pi/4,0,0,0]
    # angle = [0,0,0,0]
    pos_to_send = 0
    while 1:
        #Delta time 

        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        image = np.asanyarray(color_frame.get_data())
        #Object detection
        coordinates_pix, coordinates_meters,detected = Object_3D_recontruction( image,          color_frame,
                                                                                lower_color,    upper_color, 
                                                                                flip_cam,       distance)
        forward_kinematic = state.actual_TCP_pose[:3]
        forward_kinematic_rope = forwad_kinematic_v2(state.actual_q[0],state.actual_q[1],state.actual_q[2])
        # print(angle_base,forward_kinematic)
        global_coordinates = Camera_top_to_qlobal_coords(coordinates_meters, forward_kinematic, state.actual_q[0]) 

        angle = Angle(forward_kinematic_rope[0],forward_kinematic_rope[1],global_coordinates[0],global_coordinates[1],distance)
        # cv2.putText(image, f"Angle : {angle[0]*180/math.pi,angle[1]*180/math.pi}", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255),2)
        dt = time.perf_counter() - start_time
        start_time = time.perf_counter()
    

        if reference_point_x != 0 or reference_point_y !=0:
            error_x = angle[0]

            x_velocity = (angle[0]-prev_angle_x)/(dt)
            prev_angle_x = angle[0]

            x_velocity_pos = (global_coordinates[0]-prev_pos)/dt
            prev_pos = global_coordinates[0]

            x_acceleration = (x_velocity-prev_velocity_x)/dt
            prev_velocity_x = x_velocity
            F = find_lqr_control_input(UR_MASS,PAYLOAD_MASS,ROPE_LENGTH,GRAVITY,DEMP_COFF,PENDULUM,global_coordinates[0],x_velocity_pos,angle[0],x_velocity)
            # theta_double_dot = (((cart.mass + pendulum.ball_mass) * g * math.sin(pendulum.theta)) + (F * math.cos(pendulum.theta)) - (pendulum.ball_mass * ((theta_dot)**2.0) * pendulum.length * math.sin(pendulum.theta) * math.cos(pendulum.theta))) / (pendulum.length * (cart.mass + (pendulum.ball_mass * (math.sin(pendulum.theta)**2.0)))) 
            # x_double_dot = ((pendulum.ball_mass * g * math.sin(pendulum.theta) * math.cos(pendulum.theta)) - (pendulum.ball_mass * pendulum.length * math.sin(pendulum.theta) * (theta_dot**2)) + (F)) / (cart.mass + (pendulum.ball_mass * (math.sin(pendulum.theta)**2)))

            x_double_dot = (1/DEMP_COFF)*(-PAYLOAD_MASS**2*ROPE_LENGTH**2*GRAVITY*math.cos(angle[0])*math.sin(angle[0]) + PAYLOAD_MASS*ROPE_LENGTH**2*(PAYLOAD_MASS*ROPE_LENGTH*x_velocity**2*math.sin(angle[0]) - DEMP_COFF*x_velocity_pos)) + PAYLOAD_MASS*ROPE_LENGTH*ROPE_LENGTH*(1/DEMP_COFF)*F
            theta_double_dot = (1/DEMP_COFF)*((PAYLOAD_MASS+UR_MASS)*PAYLOAD_MASS*GRAVITY*ROPE_LENGTH*math.sin(angle[0]) - PAYLOAD_MASS*ROPE_LENGTH*math.cos(angle[0])*(PAYLOAD_MASS*ROPE_LENGTH*x_velocity**2*math.sin(angle[0]) - DEMP_COFF*x_velocity)) - PAYLOAD_MASS*ROPE_LENGTH*math.cos(angle[0])*(1/DEMP_COFF)*F
            pos_to_send = ((dt**2) * x_double_dot) + (((pos_to_send- prev_pos) * dt) / start_time)
            # print(pos_to_send)
            cv2.putText(image, f"pos_to_send : {pos_to_send}", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255),2)
            parameters_to_trajectory_x = inital_parameters_traj(Init_pose[0],pos_to_send,v_0_x,v_2_x,     0,      0.75,    0.35)
            # print(pos_to_send,Init_pose[0])
            state = con.receive()
            if state.runtime_state > 1 and detected:
                if watchdog.input_int_register_0 != 2:
                    watchdog.input_int_register_0 = 2
                    con.send(watchdog)  # sending mode == 4
                        
                #Trajectory for x
                q_x, dq_x, ddq_x = asym_trajectory(dt,parameters_to_trajectory_x)
                Init_pose[0] = q_x
                # Inverse Kinematic
                try: 
                    q1, q2, q3 = inverse_kinematic(Init_pose[0], -0.8845, 0.750)
                    q6 = q2 +q3 +math.pi/2
                    send_to_ur = [q1,q2,q3,-1.570796327,-3.141592654,q6]
                    # cv2.putText(image, f"send_to_ur: {send_to_ur}", (100,400), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255),2)

                    list_to_setp(setp, send_to_ur)
                    con.send(setp)  # sending new8 pose
                except ValueError as info:
                    print(info)

                #Update values
                v_0_y,v_0_x = state.actual_TCP_speed[1],state.actual_TCP_speed[0]
                v_2_y,v_2_x = v_0_y,v_0_x
                Init_pose[0] = pos_to_send
            else:
                if watchdog.input_int_register_0 != 4:
                    watchdog.input_int_register_0 = 4
                    con.send(watchdog)  # sending mode == 4
            
            if keyboard.is_pressed("esc"):  # Break loop with ESC-key
                state = con.receive()
                # ====================mode 3===================
                watchdog.input_int_register_0 = 3
                con.send(watchdog)
                con.send_pause()
                con.disconnect()
                pipeline.stop()
                break
        else:
            #Sending mode == 2 if no referece point
            con.send(watchdog)  # sending mode == 2
            state = con.receive()
        if keyboard.is_pressed("k") and pf:
            # reference_point_x = global_coordinates[0]
            # reference_point_y = global_coordinates[1]
            reference_point_x = 0.041353141344053074
            reference_point_y = -0.8189731696854206
            print(f"Reference point its ready: {reference_point_x}, {reference_point_y}")          
        if show_cam:
            cv2.imshow("Result", image)
            if cv2.waitKey(1):  # Break loop with ESC-key
                pass
if __name__ == '__main__':
    # try:
    main()
    # except Exception as e:
    #     print(e)
    #     pipeline.stop()

