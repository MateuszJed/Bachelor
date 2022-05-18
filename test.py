import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
path = r"C:\Users\mateusz.jedynak\OneDrive - NTNU\Programmering\Python\Prosjekt\Bachelor\Source\Bachelor\Data\Dampeing_ratio\Damp_ratio-0.csv"
AngleY_new = []

def read_csv_data_to_list(file):
    # read the csv file:
    df = pd.read_csv(file, skiprows=2)
    t = np.array(df['Time'].to_list())
    X = np.array(df["X"].to_list())
    Y = np.array(df["Y"].to_list())
    AngleX = np.array(df["AngleX"].to_list())
    AngleY = np.array(df["AngleY"].to_list())
    return t,X,Y,AngleX,AngleY

t,X,Y,AngleX,AngleY = read_csv_data_to_list(path)

for i in AngleY:
    AngleY_new.append(i-0.0554346243986452)


plt.figure("y",figsize=(15,6))
plt.plot(t, AngleY_new)
plt.grid("grid")
plt.ylabel("Angle in y-direciton [radian]")
plt.xlabel("Time[s]")
plt.show()
# if legend:
#     plt.legend(loc=loc)
