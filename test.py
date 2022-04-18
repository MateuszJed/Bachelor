import time
from matplotlib import pyplot as plt
import numpy as np
from numpy import pi

def trap_integrate(dt,a,v,q,a_prev):
    #inital values:
    q_0 = q
    v_0 = v
    #intrgrate acceralation:
    v = ((a+a_prev)/2) * dt + v_0
    #integrate velocity:
    q = ((v + v_0)/2) * dt + q_0
    return q,v,a


log_q = []
log_v = []
log_a = []
log_t = []

#rectangle integration:
def rec_integrate(dt,a,v,q):
    dv = v + a * dt
    dq = q + dv * dt
    return dq,dv,a

#trapezoid integration:
def trap_integrate(dt,a,v,q,a_prev):
    #inital values:
    q_0 = q
    v_0 = v
    #intrgrate acceralation:
    v = ((a+a_prev)/2) * dt + v_0
    #integrate velocity:
    q = ((v + v_0)/2) * dt + q_0
    return q,v,a


def log_integrate(dq,dv,a,t):
    log_q.append(dq)
    log_v.append(dv)
    log_a.append(a)
    log_t.append(t)

def plot_integrate():
    plt.figure()
    plt.plot(log_t, log_q)
    plt.grid()
    plt.ylabel('Position x[m]')
    plt.xlabel('Time [sec]')
    plt.title('position')

    plt.figure()
    plt.plot(log_t, log_v)
    print(max(log_v)/2)
    plt.grid()
    plt.ylabel('Velocity [m/s]')
    plt.xlabel('Time [sec]')
    plt.title('velocity')

    plt.figure()
    plt.plot(log_t, log_a)
    plt.grid()
    plt.ylabel('acccerelation x[m/s^2]')
    plt.xlabel('Time [sec]')
    plt.title('accerelation')
    plt.show()


t= 0
T =5
start_time= time.time()
v = -0.1591543757574222
q = 0
log_t=[]
prev_time = 0
prev_a=0

while time.time() - start_time < T:
    t = time.time() - start_time
    dt = t -prev_time
    a =  np.sin(2*pi*t)
    q,v,a = trap_integrate(dt,a,v,q,prev_a)
    prev_a = a
    prev_time = t
    log_integrate(q,v,a,t)
plot_integrate()