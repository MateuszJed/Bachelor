from tkinter import *
from tkinter import ttk

def val(value):
    print(value)
  
root = Tk()
root.geometry("200x200")
frame = Frame(root)
frame.pack()
current_value_kp = DoubleVar()
current_value_kd = DoubleVar()
current_value_ki = DoubleVar()
  
def get_current_value():
    return '{: .2f}'.format(current_value_kp.get())


def slider_changed(event):
    value_label.configure(text=get_current_value())

value_label = ttk.Label(
    root,
    text=get_current_value()
)
  
Kp_slider = ttk.Scale(
    root,
    from_=0,
    to=100,
    orient='horizontal',  # vertical
    command=slider_changed,
    variable=current_value_kp
)
Kp_slider.pack(padx=5, pady=5)

Kd_slider = ttk.Scale(
    root,
    from_=0,
    to=100,
    orient='horizontal',  # vertical
    command=slider_changed,
    variable=current_value_kp
)
Kd_slider.pack(padx=5, pady=5)

Ki_slider = ttk.Scale(
    root,
    from_=0,
    to=100,
    orient='horizontal',  # vertical
    command=slider_changed,
    variable=current_value_kp
)
Ki_slider.pack(padx=5, pady=5)
  

root.mainloop()