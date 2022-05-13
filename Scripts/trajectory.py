from matplotlib import pyplot as plt

def inital_parameters_traj(start_pos,end_pos,start_vel,end_vel,start_time,end_time,flex_point):
    # Inital parameters:
    q_0, q_1 = start_pos, end_pos
    v_0, v_1 = start_vel, end_vel
    t_0, t_1 = start_time, end_time
    t_f = flex_point
    t_d = end_time - flex_point
    t_a = (flex_point - start_time)
    h = end_pos - start_pos
    T = end_time-start_pos
    return [q_0, q_1, v_0, v_1, t_0, t_1,t_f,t_a,t_d,h,T]

def asym_trajectory(t,list_to_unpac):#,q_0,q_1,v_0,v_1,h,T,t_0,t_1,t_f,t_a,t_d):
    q_0, q_1, v_0, v_1, t_0, t_1,t_f,t_a,t_d,h,T = list_to_unpac 
    if t<t_f:
        #calculate q_a:
        q = q_0 + v_0 * (t-t_0) + ((2 * h - v_0 * (T + t_a) - v_1 * t_d) / (2 * T * t_a)) * (t - t_0) ** 2
        dq = v_0 + 2 * ((2 * h - v_0 * (T + t_a) - v_1 * t_d) / (2 * T * t_a)) * (t - t_0)
        ddq = 2*((2 * h - v_0 * (T + t_a) - v_1 * t_d) / (2 * T * t_a))

    if t>=t_f:
        q = (2 * q_1 * t_a + t_d * (2 * q_0 + t_a * (v_0 - v_1))) / (2 * T) + (2 * h - v_0 * t_a - v_1 * t_d) / (T) * (t - t_f) - (2 * h - v_0 * t_a - v_1 * (T + t_d)) / (2 * T * t_d) * (t - t_f) ** 2
        dq = (2 * h - v_0 * t_a - v_1 * t_d) / (T) + 2 * (- (2 * h - v_0 * t_a - v_1 * (T + t_d)) / (2 * T * t_d)) * (t - t_f)
        ddq = 2*(- (2 * h - v_0 * t_a - v_1 * (T + t_d)) / (2 * T * t_d))

    return q

# log_q= []
# log_dq= []
# log_ddq =[]
# log_t = []
# def log_traj(q,dq,ddq,t):
#     log_q.append(q)
#     log_dq.append(dq)
#     log_ddq.append(ddq)
#     log_t.append(t)

# def plot_traj():
#     # plot figures:
#     plt.figure()
#     plt.plot(log_t, log_q)
#     plt.grid()
#     plt.ylabel('Position x[m]')
#     plt.xlabel('Time [sec]')
#     plt.title('position')

#     plt.figure()
#     plt.plot(log_t, log_dq)
#     plt.grid()
#     plt.ylabel('Velocity [m/s]')
#     plt.xlabel('Time [sec]')
#     plt.title('velocity')

#     plt.figure()
#     plt.plot(log_t, log_ddq)
#     plt.grid()
#     plt.ylabel('acccerelation x[m/s^2]')
#     plt.xlabel('Time [sec]')
#     plt.title('accerelation')
#     plt.show()