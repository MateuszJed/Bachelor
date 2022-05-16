import math,time,keyboard,os,csv
from Kinematic import inverse_kinematic,forwad_kinematic
from UR10 import initial_communiation

def main():
    # Client has a few methods to get proxy to UA nodes that should always be in address space such as Root or Objects
    setp,con,watchdog,Init_pose = initial_communiation('169.254.182.10', 30004,500)
    path = r"C:\Users\mateusz.jedynak\OneDrive - NTNU\Programmering\Python\Prosjekt\Bachelor\Source\Bachelor\Data\Kinematic"
    count = 0.5
    run = False
    pf = True
    watchdog.input_int_register_0 = 2
    con.send(watchdog)  # sending mode == 2
    state = con.receive()
    log_x,log_y,log_t = [], [], []
    # Init_pose = 0.2397127693021015 -0.46028723069789845
    while state.runtime_state > 1:
        #Inverse Kinematic
        if run:
            try: 
                q1, q2, q3 = inverse_kinematic(Init_pose[0], Init_pose[1], Init_pose[2])
                q6 = q2 +q3 +math.pi/2
                send_to_ur = [q1,q2,q3,-1.570796327,-3.141592654,q6]
                list_to_setp(setp, send_to_ur)
                con.send(setp)  # sending new8 pose
            except ValueError as info:
                print(info)

            x = 0.5*math.sin(count)
            y = -0.7+0.5*math.sin(count)
            count += .05
            time.sleep(0.12)
            
            log_x.append(Init_pose[0])
            log_y.append(Init_pose[1])
            log_t.append(time.time()- start_timer)

        if keyboard.is_pressed("esc"):  # Break loop with ESC-key
            info_csv_1 = [f"Kinematic test"]
            info_csv_2 = [f"x: 0.5*math.sin(count), y: -0.7+0.5*math.sin(count)"]
            header = ["Time","X","Y"]
            with open(path + '\Kinematic_test_{}.csv'.format(str(len(os.listdir(path)))), 'w',newline="") as f:
                # create the csv writer
                writer = csv.writer(f)
                writer.writerow(info_csv_1)
                writer.writerow(info_csv_2)
                writer.writerow(header)
                for i in range(len(log_time)):
                    writer.writerow([log_t,log_x,log_y])
            state = con.receive()
            # ====================mode 3===========k========
            watchdog.input_int_register_0 = 3
            con.send(watchdog)
            break
        if keyboard.is_pressed("k") and pf:
            run = True
            start_timer = time.time()
            pf = False
            
        
if __name__ == '__main__':
    main()

