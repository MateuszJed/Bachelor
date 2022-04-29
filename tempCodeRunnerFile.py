t_1, x_1, y_1 = read_csv_data_to_list(files[0])
t_2, x_2, y_2 = read_csv_data_to_list(files[1])
t_3, x_3, y_3 = read_csv_data_to_list(files[2])

plt.figure(0)
plt.plot(t_1, x_1,label="label")
plt.grid("grid")
plt.ylabel("y_axis")
plt.xlabel("x_axis")
plt.title("title")
plt.legend(loc="loc")

