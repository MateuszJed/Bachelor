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
    return t,X,Y

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
x_y_meter = "X-Y-angle-PID-cam-top"


files = find_csv_files(x_y_meter)

format_x = ["X","pixel"]
format_y = ["Y","m"]
format = format_y
for n, file in enumerate(files):
    t, x, y = read_csv_data_to_list(file)
    plot_graps(t, x, 1,"PID-controller in {}-direction ".format(format_x[0]), "Time[s]", "Error from ref [{}]".format(format_x[1]), f"Trail: {n}")
# x = np.ones(11)*0.043303780712741435
# plt.plot(x, color = 'blue', linewidth=1, linestyle='-.')
for n, file in enumerate(files):
    t, x, y = read_csv_data_to_list(file)
    plot_graps(t, y, 2,"PID-controller in {}-direction ".format(format_y[0]), "Time[s]", "Error from ref [{}]".format(format_y[1]), f"Trail: {n}")
# x = np.ones(11)*-0.7977911479915556
# plt.plot(x, color = 'blue', linewidth=1, linestyle='-.')

plt.show()