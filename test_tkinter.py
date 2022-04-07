from tkinter import *
from tkinter import ttk

def Mordi():  
    root = Tk()
    global newKp, newKd, newKi
    root.geometry("200x200")
    frame = Frame(root)
    frame.pack()
    current_value_kp = DoubleVar()
    current_value_kd = DoubleVar()
    current_value_ki = DoubleVar()
    
    def get_current_value_kp():
        print(f"Kp: {(current_value_kp.get())}")
        newKp = current_value_kp.get()
        
    def get_current_value_kd():
        print(f"Kp: {(current_value_kp.get())}")
    def get_current_value_ki():
        print(f"Kp: {(current_value_kp.get())}")


    def slider_changed_kp(event):
        value_label_kp.configure(text=get_current_value_kp())
    def slider_changed_kd(event):
        value_label_kd.configure(text=get_current_value_kd())
    def slider_changed_ki(event):
        value_label_ki.configure(text=get_current_value_ki())


    value_label_kp = ttk.Label(
        root,
        text=get_current_value_kp()
    )
    value_label_kd = ttk.Label(
        root,
        text=get_current_value_kd()
    )
    value_label_ki = ttk.Label(
        root,
        text=get_current_value_ki()
    )


    Kp_slider = ttk.Scale(
        root,
        from_=0,
        to=100,
        orient='horizontal',  # vertical
        command=slider_changed_kp,
        variable=current_value_kp
    )
    Kp_slider.pack(padx=5, pady=5)

    Kd_slider = ttk.Scale(
        root,
        from_=0,
        to=100,
        orient='horizontal',  # vertical
        command=slider_changed_kd,
        variable=current_value_kd)
    Kd_slider.pack(padx=5, pady=5)

    Ki_slider = ttk.Scale(
        root,
        from_=0,
        to=100,
        orient='horizontal',  # vertical
        command=slider_changed_ki,
        variable=current_value_ki
    )
    Ki_slider.pack(padx=5, pady=5)
    return root.mainloop()

# root.mainloop()

# if __name__ == "__main__":
#     Mordi()




# def Mordi():  
#     global newKp, newKd, newKi  
#     root = Tk()
#     root.geometry("200x200")
#     frame = Frame(root)
#     frame.pack()
#     current_value_kp = DoubleVar()
#     current_value_kd = DoubleVar()
#     current_value_ki = DoubleVar()
    
#     def get_current_value_kp():
#         # print(f"Kp: {(current_value_kp.get())}")
#         newKp = current_value_kp.get()
        
#     def get_current_value_kd():
#         print(f"Kp: {(current_value_kp.get())}")
#     def get_current_value_ki():
#         print(f"Kp: {(current_value_kp.get())}")


#     def slider_changed_kp(event):
#         value_label_kp.configure(text=get_current_value_kp())
#     def slider_changed_kd(event):
#         value_label_kd.configure(text=get_current_value_kd())
#     def slider_changed_ki(event):
#         value_label_ki.configure(text=get_current_value_ki())


#     value_label_kp = ttk.Label(
#         root,
#         text=get_current_value_kp()
#     )
#     value_label_kd = ttk.Label(
#         root,
#         text=get_current_value_kd()
#     )
#     value_label_ki = ttk.Label(
#         root,
#         text=get_current_value_ki()
#     )


#     Kp_slider = ttk.Scale(
#         root,
#         from_=0,
#         to=100,
#         orient='horizontal',  # vertical
#         command=slider_changed_kp,
#         variable=current_value_kp
#     )
#     Kp_slider.pack(padx=5, pady=5)

#     Kd_slider = ttk.Scale(
#         root,
#         from_=0,
#         to=100,
#         orient='horizontal',  # vertical
#         command=slider_changed_kd,
#         variable=current_value_kd)
#     Kd_slider.pack(padx=5, pady=5)

#     Ki_slider = ttk.Scale(
#         root,
#         from_=0,
#         to=100,
#         orient='horizontal',  # vertical
#         command=slider_changed_ki,
#         variable=current_value_ki
#     )
#     Ki_slider.pack(padx=5, pady=5)
    root.mainloop()
    return current_value_kp.get() #root.mainloop()