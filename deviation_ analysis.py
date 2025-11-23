import pandas as pd
import numpy as np

# PID+TinyMPC ve PID+TinyMPC+VIO CSV dosyalarını yükleme
pid_data = pd.read_csv("test45_emre_pid_pose_data_outputmpc025635.csv")
vio_data = pd.read_csv("test45_emre_pid_pose_data_vio_output02587.csv")

# X ve Y ekseni değerlerini çekme
x_pid = pid_data["field.values0"]
x_vio = vio_data["field.values0"]
y_pid = pid_data["field.values1"]
y_vio = vio_data["field.values1"]

# Ortalama ve standart sapma hesaplama
mean_x_pid, std_x_pid = np.mean(x_pid), np.std(x_pid)
mean_x_vio, std_x_vio = np.mean(x_vio), np.std(x_vio)
mean_y_pid, std_y_pid = np.mean(y_pid), np.std(y_pid)
mean_y_vio, std_y_vio = np.mean(y_vio), np.std(y_vio)

print(f"PID+TinyMPC - X Ortalama: {mean_x_pid}, X Standart Sapma: {std_x_pid}")
print(f"PID+TinyMPC+VIO - X Ortalama: {mean_x_vio}, X Standart Sapma: {std_x_vio}")
print(f"PID+TinyMPC - Y Ortalama: {mean_y_pid}, Y Standart Sapma: {std_y_pid}")
print(f"PID+TinyMPC+VIO - Y Ortalama: {mean_y_vio}, Y Standart Sapma: {std_y_vio}")
