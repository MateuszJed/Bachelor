from cProfile import label
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import glob
#________________________Functions_________________________
def find_csv_files(folder):
    path = r"C:\Users\mateusz.jedynak\OneDrive - NTNU\Programmering\Python\Prosjekt\Bachelor\Source\Bachelor\Data\{}\*.csv".format(folder)
    # Prints all types of txt files present in a Path
    files = []
    for file in glob.iglob(path, recursive=True):
        files.append(file)
    return files
def read_csv_data_to_list_angle(file,text):
    # read the csv file:
    df = pd.read_csv(file, skiprows=2)
    t = np.array(df['Time'].to_list())
    X = np.array(df["X"].to_list())
    Y = np.array(df["Y"].to_list())
    if text == "angle":
        AngleX = np.array(df["AngleX"].to_list())
        AngleY = np.array(df["AngleY"].to_list())
        End_effector_X = np.array(df["End_effector_X"].to_list())
        End_effector_Y = np.array(df["End_effector_Y"].to_list())
        return t,X,Y,AngleX,AngleY,End_effector_X,End_effector_Y
    if text == "pos":
        End_effector_X = np.array(df["End_effector_X"].to_list())
        End_effector_Y = np.array(df["End_effector_Y"].to_list())
        return t,X,Y,End_effector_X,End_effector_Y

folder = "angle_pos_PID_endeffect_angle_v2"
files = find_csv_files(folder)
# angle_PID, module_PID, pos_PID = files[0],files[1],files[2]
module_PID = files[2]
# t_angle,X_angle,Y_angle,AngleX_angle,AngleY_angle,End_effector_X_angle,End_effector_Y_angle = read_csv_data_to_list_angle(angle_PID,"angle")
t_module,X_module,Y_module,AngleX_module,AngleY_module,End_effector_X_module,End_effector_Y_module = read_csv_data_to_list_angle(module_PID,"angle")
# t_pos,X_pos,Y_pos,End_effector_X_pos,End_effector_Y_pos = read_csv_data_to_list_angle(pos_PID,"pos")


# plt.figure(1,figsize=(13,6))
# plt.plot(t_angle, X_angle,label="Payload position [meter]")
# plt.plot(t_angle, AngleX_angle,label="Angle of wire [radian]")
# plt.plot(t_angle, End_effector_X_angle,label="End effector[meter]")
# plt.grid(True)
# plt.legend(loc='upper right')
# plt.xlabel("Time [s]")
# plt.title("PID controller with respect to the angle in x-direction")

# plt.figure(2,figsize=(13,6))
# plt.plot(t_angle, Y_angle,label="Payload position [meter]")
# plt.plot(t_angle, AngleY_angle,label="Angle of wire [radian]")
# plt.plot(t_angle, End_effector_Y_angle,label="End effector[meter]")
# plt.xlabel("Time [s]")
# plt.grid(True)
# plt.legend(loc='upper right')
# plt.title("PID controller with respect to the angle in y-direction")

plt.figure(3,figsize=(13,6))
plt.plot(t_module, X_module,label="Payload position [meter]")
plt.plot(t_module, AngleX_module,label="Angle of wire [radian]")
plt.plot(t_module, End_effector_X_module,label="End effector[meter]")
plt.xlabel("Time [s]")
plt.grid(True)
plt.legend(loc='upper right')
plt.title("Module PID controller in x-direction")

plt.figure(4,figsize=(13,6))
plt.plot(t_module, Y_module,label="Payload position [meter]")
plt.plot(t_module, AngleY_module,label="Angle of wire [radian]")
plt.plot(t_module, End_effector_Y_module,label="End effector[meter]")
plt.xlabel("Time [s]")
plt.grid(True)
plt.legend(loc='upper right')
plt.title("Module PID controller in y-direction")

# plt.figure(5,figsize=(13,6))
# plt.plot(t_pos, X_pos,label="Payload position [meter]")
# plt.plot(t_pos, End_effector_X_pos,label="End effector[meter]")
# plt.xlabel("Time [s]")
# plt.grid(True)
# plt.legend(loc='upper right')
# plt.title("PID controller with respect to the position in x-direction")

# plt.figure(6,figsize=(13,6))
# plt.plot(t_pos, Y_pos,label="Payload position [meter]")
# plt.plot(t_pos, End_effector_Y_pos,label="End effector[meter]")
# plt.xlabel("Time [s]")
# plt.grid(True)
# plt.legend(loc='upper right')
# plt.title("PID controller with respect to the position in y-direction")

# plt.ylabel(y_axis)
# plt.xlabel(x_axis)
# plt.title(title)
# if legend:
#     plt.legend(loc=loc)

plt.show()