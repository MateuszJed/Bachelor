import math
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