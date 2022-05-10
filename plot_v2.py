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

def read_csv_data_to_list(file):
    # read the csv file:
    df = pd.read_csv(file, skiprows=2)
    t = np.array(df['Time'].to_list())
    X = np.array(df["X"].to_list())
    Y = np.array(df["Y"].to_list())
    End_effector_X = np.array(df["End_effector_X"].to_list())
    End_effector_Y = np.array(df["End_effector_Y"].to_list())
    return t,X,Y,End_effector_X,End_effector_Y

def plot_graps(x,y,fig_num=1,title="",x_axis="x",y_axis="y",label="x",legend=True,grid=True,loc ='upper right'):
    plt.figure(fig_num,figsize=(13,6))
    plt.plot(x, y,label=label)
    plt.grid(grid)
    plt.ylabel(y_axis)
    plt.xlabel(x_axis)
    plt.title(title)
    if legend:
        plt.legend(loc=loc)

#_______________________Code_________________________

#different folders:
x_y_meter = "testing_2\pos"


files = find_csv_files(x_y_meter)

format_x = ["X","m"]
format_y = ["Y","m"]
format = format_y

# t_1, x_1, y_1 = read_csv_data_to_list(files[0])
# t_2, x_2, y_2 = read_csv_data_to_list(files[1])
# t_3, x_3, y_3 = read_csv_data_to_list(files[2])
# t_4, x_4, y_4 = read_csv_data_to_list(files[3])

# plt.figure(0)
# plt.plot(t_4, y_4,label="Kp:0.5,  Kd:0.005, Ki: 0.006")
# plt.plot(t_1, y_1,label="Kp:0.55, Kd:0.007, Ki: 0.006")
# plt.plot(t_2, y_2,label="Kp:0.4,  Kd:0.002, Ki: 0.001")
# plt.plot(t_3, y_3,label="Kp:0.5,  Kd:0.006, Ki: 0.009")
# plt.grid("grid")
# plt.ylabel("Payload position in y-direction [meter]")
# plt.xlabel("Time [seconds]")
# plt.title("System response with different PID values")
# # plt.legend("Kp_x:0.55, Kp_y:0.55, Kd_x:0.007")
# plt.legend()


for n, file in enumerate(files):
    t, x, y,End_effector_X,End_effector_Y = read_csv_data_to_list(file)
    plot_graps(t, x, 1,"PID-controller in x-direction ", "Time[s]", "Payload position", f"Trail: {n}")
# x = np.ones(11)*0.043303780712741435
# plt.plot(x, color = 'blue', linewidth=1, linestyle='-.')
for n, file in enumerate(files):
    t, x, y,End_effector_X,End_effector_Y = read_csv_data_to_list(file)
    plot_graps(t, y, 2,"PID-controller in y-direction ", "Time[s]", "Payload position", f"Trail: {n}")
# x = np.ones(11)*-0.7977911479915556
# plt.plot(x, color = 'blue', linewidth=1, linestyle='-.')
for n, file in enumerate(files):
    t, x, y,End_effector_X,End_effector_Y = read_csv_data_to_list(file)
    plot_graps(t, End_effector_X, 3,"PID-controller in x-direction ", "Time[s]", "End effetor", f"Trail: {n}")

for n, file in enumerate(files):
    t, x, y,End_effector_X,End_effector_Y = read_csv_data_to_list(file)
    plot_graps(t, End_effector_Y, 4,"PID-controller in y-direction ", "Time[s]", "End effetor", f"Trail: {n}")

plt.show()