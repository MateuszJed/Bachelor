import math
import numpy as np
from math import asin, acos, atan2, pi
from cmath import sqrt

def inverse_kinematic(x_position, y_position,z_position):

    d = [0.1273]
    a = [0,0,0.612, 0.688]

    r1 = sqrt(x_position**2+y_position**2)
    r1 = r1.real
    r2 = z_position-d[0]
    r3 = sqrt(r2**2+r1**2)
    r3 = r3.real

    omega_1 = -atan2(r2,r1)
    omgea_2 = -acos(((a[2]**2)+(r3**2)-(a[3]**2))/(2*a[2]*r3))
    omgea_3 = acos(((a[2]**2)+(a[3]**2)-(r3**2))/(2*a[2]*a[3]))

    q1 = atan2(y_position, x_position)
    q1 = q1.real

    if y_position <= 0:
        q1 = q1 +pi
    else:
        q1 = q1 -pi

    q2 = omega_1+omgea_2
    q2 = q2.real

    q3 = pi-omgea_3
    q3 = q3.real

    return q1,q2,q3
def forwad_kinematic(q1, q2, q3):

    #D_H Parameters
    alpha = np.array([0, np.pi/2, 0, 0])
    d = np.array([0.1273, 0, 0, 0.0127])
    a = np.array([0, 0, -0.612, -0.688])
    q = np.array([q1, 0, q2, q3])

    # Homogonous Transformations


    T0 = np.array([[np.cos(q[0]), -np.sin(q[0])*np.cos(alpha[0]), np.sin(q[0])*np.sin(alpha[0]), a[0]*np.cos(q[0])],
                   [np.sin(q[0]), np.cos(q[0])*np.cos(alpha[0]), -np.cos(q[0])*np.sin(alpha[0]), a[0]*np.sin(q[0])],
                   [0, np.sin(alpha[0]), np.cos(alpha[0]), d[0]],
                   [0, 0, 0, 1]])

    T1 = np.array([[np.cos(q[1]), -np.sin(q[1])*np.cos(alpha[0]), np.sin(q[1])*np.sin(alpha[0]), a[1]*np.cos(q[1])],
                   [np.sin(q[1]), np.cos(q[1])*np.cos(alpha[1]), -np.cos(q[1])*np.sin(alpha[1]), a[1]*np.sin(q[1])],
                   [0, np.sin(alpha[1]), np.cos(alpha[1]), d[1]],
                   [0, 0, 0, 1]])

    T2 = np.array([[np.cos(q[2]), -np.sin(q[2])*np.cos(alpha[2]), np.sin(q[2])*np.sin(alpha[2]), a[2]*np.cos(q[2])],
                   [np.sin(q[2]), np.cos(q[2])*np.cos(alpha[2]), -np.cos(q[2])*np.sin(alpha[2]), a[2]*np.sin(q[2])],
                   [0, np.sin(alpha[2]), np.cos(alpha[2]), d[2]],
                   [0, 0, 0, 1]])

    T3 = np.array([[np.cos(q[3]), -np.sin(q[3])*np.cos(alpha[3]), np.sin(q[3])*np.sin(alpha[3]), a[3]*np.cos(q[3])],
                   [np.sin(q[3]), np.cos(q[3])*np.cos(alpha[3]), -np.cos(q[3])*np.sin(alpha[3]), a[3]*np.sin(q[3])],
                   [0, np.sin(alpha[3]), np.cos(alpha[3]), d[3]],
                   [0, 0, 0, 1]])

    T4 = T0@T1@T2@T3
    x, y, z = T4[0][3], T4[1][3], T4[2][3]
    return [x,y,z]