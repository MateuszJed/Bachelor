import math,time,keyboard,os,csv
from Scripts.Kinematic import inverse_kinematic,forwad_kinematic,forwad_kinematic_v2
from Scripts.UR10 import initial_communiation
from Scripts.miscellaneous import list_to_setp

def main():
    # Client has a few methods to get proxy to UA nodes that should always be in address space such as Root or Objects
    setp,con,watchdog,Init_pose = initial_communiation('169.254.182.10', 30004,500,"kinematic")
    path = r"C:\Users\mateusz.jedynak\OneDrive - NTNU\Programmering\Python\Prosjekt\Bachelor\Source\Bachelor\Data\Kinematic"
    count = 0.5
    count_2 = 0.5
    run = False
    pf = True
    flip = 1
    watchdog.input_int_register_0 = 2
    con.send(watchdog)  # sending mode == 2
    state = con.receive()
    log_x,log_y,log_z,log_t = [], [], [],[]
    # Init_pose = 0.2397127693021015 -0.46028723069789845
    Init_pose[0], Init_pose[1], Init_pose[2] = 0,-0.9,0.04
    while state.runtime_state > 1:
        if watchdog.input_int_register_0 != 2:
            watchdog.input_int_register_0 = 2
            con.send(watchdog)  # sending mode == 4
        state = con.receive()
        #Inverse Kinematic

        if run:
            try: 
                q1, q2, q3 = inverse_kinematic(Init_pose[0], Init_pose[1], Init_pose[2])
                send_to_ur = [q1,q2,q3,-1.570796327,-3.141592654,1.57]
                # print(send_to_ur[0]*180/math.pi,send_to_ur[1]*180/math.pi,send_to_ur[2]*180/math.pi, Init_pose[0], Init_pose[1], Init_pose[2])
                list_to_setp(setp, send_to_ur)
                con.send(setp)  # sending new8 pose
            except ValueError as info:
                print(info)

            # x = 0.5*math.sin(count)
            # y = -0.5+0.35*math.sin(count)
            # z = 0.6+0.4*math.sin(count_2)
            # count += .005
            # count_2 += .01

            x = 0.4*math.sin(count)*math.cos(count_2)
            y = -0.9+0.4*math.sin(count)*math.sin(count_2)
            z = 0.06+0.3*math.cos(count)
            # z = 0.06
            print(x,y,z)
            count += .009*flip
            count_2 += .05*flip
            # if count > math.pi:
            #     flip = -1
            # if count < 0:
            #     flip = 1

            # if count_2 > 2*math.pi:
            #     flip = -1
            # if count_2 < 0:
            #     flip = 1
            time.sleep(0.01)
            Init_pose[0], Init_pose[1],Init_pose[2] = x,y,z
            log_x.append(Init_pose[0])
            log_y.append(Init_pose[1])
            log_z.append(Init_pose[2])
            end_time = time.time()- start_timer
            log_t.append(end_time)

        if keyboard.is_pressed("esc"):  # Break loop with ESC-key
            info_csv_1 = [f"Kinematic test"]
            info_csv_2 = [f"x: 0.5*math.sin(count), y: -0.7+0.5*math.sin(count)"]
            header = ["Time","X","Y"]

            with open(path + '\X-Y-ulike_PID_{}.csv'.format(str(len(os.listdir(path)))), 'w',newline="") as f:
                # create the csv writer
                writer = csv.writer(f)
                writer.writerow(info_csv_1)
                writer.writerow(info_csv_2)
                writer.writerow(header)
                for i in range(len(log_t)):
                    writer.writerow([log_t[i],log_x[i],log_y[i],log_z[i]])
            state = con.receive()
            # ====================mode 3===========k========
            watchdog.input_int_register_0 = 3
            con.send(watchdog)
            break
        if keyboard.is_pressed("k") and pf:
            run = True
            print("Start")
            start_timer = time.time()
            pf = False
            
        
if __name__ == '__main__':
    main()

