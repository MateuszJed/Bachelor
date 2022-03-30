import math
from math import asin, acos, atan2, pi
from cmath import sqrt

def inverse_kinematic(x_cord, y_cord,z_cord):
    alpha = [0,pi/2, 0, 0]
    d = [0.128,0,0,0.0125]
    a = [0,0,-0.612,-0.68759]

    s3pn = 1
    q2pn = -1

    q1 = atan2(y_cord,x_cord)
    if y_cord >= 0:
        q1 = q1 - pi
    else:
        q1 = q1 + pi

    c3 = ((x_cord**2+y_cord**2+z_cord**2-(d[0]**2+a[2]**2+a[3]**2)-2*d[0]*(z_cord-d[0]))/(2*a[2]*a[3]))

    s3 = sqrt(1-c3)



    s3 = s3.real
    q3 = atan2(s3,c3)

    k1 = (c3*a[3]+a[2])
    k2 = (s3*a[3])

    A = -2*k1*(d[0]-z_cord)
    B = (2*k1*(d[0]-z_cord))**2-4*(k1**2+k2**2)*(z_cord**2+d[0]**2-k2**2-2*z_cord*d[0])#Kanskje slutt her e fukka -2*z_cord*d[0]
    q2 = asin((A-math.sqrt(B))/(2*(k1**2+k2**2)))

    return q1,q2,q3
    #angle = [q1*180/pi,q2*180/pi,q3*180/pi]
