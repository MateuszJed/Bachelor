import cmath
def _map(x, in_min, in_max, out_min, out_max):
    return float((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def setp_to_list(setp):
    temp = []
    for i in range(0, 6):
        temp.append(setp.__dict__["input_double_register_%i" % i])
    return temp


def list_to_setp(setp, list):
    for i in range(0, 6):
        setp.__dict__["input_double_register_%i" % i] = list[i]
    return setp

def Angle(UR_10_x,UR_10_y,object_x,object_y,l):
    return cmath.asin((object_x-UR_10_x)/l).real,cmath.asin((object_y-UR_10_y)/l).real

def PID(Kp,Kd,Ki,messurment, referance_point, prev_error,integral,delta_time,flip_error):
    error = (messurment-referance_point)*flip_error
    dedt = (error-prev_error)/delta_time    #Derivative
    integral = integral + error*delta_time  #Integral

    return Kp*error + Kd*dedt + Ki*integral,error,integral