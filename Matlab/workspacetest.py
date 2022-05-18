L1 = 128
L2 = 612
L3 = 688
x_cord = 618.2
y_cord = -852
z_cord = 891
# x_cord = 1300
# y_cord = 1400
# z_cord = 1500



#THIS LINE TO CONTSTRAIN WORKING AREA:
f = (-L2+L3)**2 <= x_cord**2+y_cord**2+(z_cord-L1)**2 <= (L2+L3)**2




print(f)
if f :
    print('yoo')
else :
    print('shiiii')