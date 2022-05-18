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
    AngleX = np.array(df["AngleX"].to_list())
    AngleY = np.array(df["AngleY"].to_list())
    return t,X,Y,AngleX,AngleY

def plot_graps(x,y,fig_num=1,x_axis="x",y_axis="y",label="x",legend=True,grid=True,loc ='upper right'):
    plt.figure(fig_num,figsize=(15,6))
    plt.plot(x, y,label=label)
    plt.grid(grid)
    plt.ylabel(y_axis)
    plt.xlabel(x_axis)
    # if legend:
    #     plt.legend(loc=loc)

#different folders:
x_y_meter = "Dampeing_ratio"
files = find_csv_files(x_y_meter)

for n, file in enumerate(files):
    t, x, y,AngleX,AngleY = read_csv_data_to_list(file)
    plot_graps(t, AngleX, "x", "Time[s]", "Angle in x-direciton [radian]", f"Trail: {n+1}")

for n, file in enumerate(files):
    t, x, y,AngleX,AngleY  = read_csv_data_to_list(file)
    plot_graps(t, AngleY, "y", "Time[s]", "Angle in y-direciton [radian]", f"Trail: {n+1}")

plt.show()