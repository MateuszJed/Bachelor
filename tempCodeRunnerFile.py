
                    # cv2.putText(image, f"send_to_ur: {send_to_ur}", (100,400), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255),2)

                    list_to_setp(setp, send_to_ur)
                    con.send(setp)  # sending new8 pose
                except ValueError as info:
                    print(info)


                #Update values
                v_0_y,v_0_x = state.actual_TCP_speed[1],state.actual_TCP_speed[0]
                v_2_y,v_2_x = v_0_y,v_0_x
                Init_pose[0] = pos_to_send
            else:
                if watchdog.input_int_register_0 != 4:
                    watchdog.input_int_register_0 = 4
                    con.send(watchdog)  # sending mode == 4
            
            if keyboard.is_pressed("esc"):  # Break loop with ESC-key
                state = con.receive()
                # ====================mode 3===================
                watchdog.input_int_register_0 = 3
                con.send(watchdog)
                con.send_pause()
                con.disconnect()
                pipeline.stop()
                break
        else:
            #Sending mode == 2 if no referece point
            con.send(watchdog)  # sending mode == 2
            state = con.receive()
        if keyboard.is_pressed("k") and pf:
            # reference_point_x = global_coordinates[0]
            # reference_point_y = global_coordinates[1]
            reference_point_x = 0.041353141344053074
            reference_point_y = -0.8189731696854206
            print(f"Reference point its ready: {reference_point_x}, {reference_point_y}")          
        if show_cam: