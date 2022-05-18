#calculate the damping data from the simulated system
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from statistics import mean, median
from numpy import pi
import scipy.stats as stats
import math
from scipy.integrate import odeint

#read the csv file:
df = pd.read_csv('data_angle_x_linear_long.csv')

#convert the data to np lists:
theta_rad = np.array(df['angle'].to_list())
t = np.array(df["time"].to_list())

#list comprehetnsion from grades to rad:
#theta_rad = np.array([x*(np.pi/180) for x in theta])

#plot the imported pendulum data:
plt.figure(figsize=(15,7))
plt.plot(t,theta_rad)
plt.xlabel("time [sec]")
plt.ylabel("angle [rad]")
plt.show()

#find the peaks in the data set:
peaks = []
n = 10
E = 0

#for i in range(n,len(theta)- n):  #code without filtering
 #   if theta[i] == max(theta[i-n:i+n]):
  #      peaks.append(i)


for i in range(n,len(theta_rad)-n):
    if i-E > 20 or i<20:
        if theta_rad[i] == max(theta_rad[i-n:i+n]):
            peaks.append(i)
            E=i

#print(peaks[0]-peaks[1])
#print(peaks)


#plot the peaks:
plt.figure(figsize=(15,7))
plt.title('Peak identification')
plt.plot(t,theta_rad)
plt.plot(t[peaks], theta_rad[peaks], "o")
plt.xlabel("time [sec]")
plt.ylabel("angle [rad]")
plt.show()

#find the period:
T = t[peaks[1:]]-t[peaks[0:np.size(peaks)-1]]
#print(T)

#plot the calculated periods:
plt.figure()
plt.title("periods")
plt.gca().axes.get_xaxis().set_visible(False)
plt.ylabel("time [sec]")
plt.plot(T,"o")
plt.show()

#avrage of period
T_avr = mean(T)
#print(T_avr)

#calculate logaritmic decrement:
window = 10
damping_list =[]
#
for n in range(1,len(peaks)-window-1): #HERE YOU SPESEFY WHERE TO CALCULATE DAMPING
    N = 1
    for i in range(n+1,n+window):
        #print(i)
        delta = 1/N * np.log(theta_rad[peaks[n]] / theta_rad[peaks[i]])
        damping_list.append(1/np.sqrt(1 + ((2*pi)/delta)**2))
        N += 1
        #damping_ratio = 1/np.sqrt(1 + ((2*pi)/delta)**2)



plt.figure(6)
mu = mean(damping_list)
variance = np.var(damping_list)
sigma = math.sqrt(variance)
x = np.linspace(mu - 3*sigma, mu + 3*sigma, 100)
plt.title(f"mean={round(mu,5)}, variance={round(variance,8)}")
plt.plot(x, stats.norm.pdf(x, mu, sigma))
plt.show()
#print(damping_list)

#damping_avg = mean(damping_list)
damping_avg = mean(damping_list)
print(damping_avg)
#w_d = (2*pi)/T_avr
#w_n = w_d/(np.sqrt(1 - delta**2))

#print("damping ratio:",damping_ratio)
#print("natural_frequence:",w_n)

#=======================================================================
#plot the model with new damping coeficcient

# pendulum hypeparameters:
g: float = 9.81  # gravity [kg/m*s^2]
L: int = 1.268  # length rope [m]

# inital values [angle,angle_velocity]
init = [theta_rad[peaks[0]], 0]

# second order: dx^2 + lam*dx + k*x = 0
#damped
zeta, wn = 2 * damping_avg * np.sqrt(g / L), g / L

#undamped
#zeta, wn = 0, g / L


print(f"damping coe: {damping_avg}")
#first order ODE
def shm(init, t):
    global zeta, wn
    x = init[0]
    y = init[1]
    dxdt = y
    dydt = -wn * x - zeta * y
    return np.array([dxdt, dydt])


#solve the first order ODE
solve = odeint(shm, init, t[peaks[0]:])

#store the solutions:
header = ['angle','angle_velocity','time']
theta_est = solve[:, 0]
dot_theta = solve[:, 1]

#plot angle:
plt.figure(figsize=(15,7))
plt.plot( t[peaks[0]:],theta_rad[peaks[0]:],label='real system')
plt.plot( t[peaks[0]:], theta_est,"--",label='damped model')
plt.xlabel("time [sec]")
plt.ylabel("angle [rad]")
plt.legend(loc=1)
plt.show()