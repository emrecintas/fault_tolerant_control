import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from PIL import Image

# CSV dosyalarının yolları
csv_file_paths = ['test_emre_pid_pose_data_outputmpc01011002_05.csv', 'test_emre_pid_pose_data_vio_output01011003_05.csv']

# Her bir CSV dosyası için etiketler
labels = ['PID+TinyMPC', 'PID+TinyMPC+VIO']

# Her veri kümesi için renkler
colors = ['b', 'g']  # 'b' mavi (sabit), 'g' yeşil (canlı)

# Sabit grafik için veri yükle (mavi grafik)
mpc_data = pd.read_csv(csv_file_paths[0])
mpc_data['%time'] = mpc_data['%time'] / 1e9  # Zaman damgasını saniyeye çevir
mpc_data['field.header.stamp'] = mpc_data['field.header.stamp'] / 1e9

# Canlı çizim için veri yükle (yeşil grafik)
vio_data = pd.read_csv(csv_file_paths[1])
vio_data['%time'] = vio_data['%time'] / 1e9  # Zaman damgasını saniyeye çevir
vio_data['field.header.stamp'] = vio_data['field.header.stamp'] / 1e9

# 3B Grafik oluştur
fig = plt.figure(figsize=(14, 10), dpi=200)  # Daha yüksek DPI ile yüksek çözünürlük
ax = fig.add_subplot(111, projection='3d')

# Sabit grafiği çiz (mavi)
def draw_static():
    ax.plot(mpc_data['field.values0'], mpc_data['field.values1'], mpc_data['field.values2'],
            label=labels[0], color=colors[0])

# Canlı grafik için başlangıç ayarları
x_data, y_data, z_data = [], [], []

# Bellekte çerçeve biriktir
frames = []

def update(frame):
    # Grafik eksenini temizle
    ax.clear()

    # Sabit grafiği tekrar çiz (mavi)
    draw_static()

    # Canlı veri güncelle
    x_data.append(vio_data['field.values0'].iloc[frame])
    y_data.append(vio_data['field.values1'].iloc[frame])
    z_data.append(vio_data['field.values2'].iloc[frame])

    # Canlı grafiği çiz (yeşil)
    ax.plot(x_data, y_data, z_data, label=labels[1], color=colors[1])

    # Eksen ayarları
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.set_title('Real-Time Z-Axis Hovering with Fault-Tolerant Control (vy=-0.3, yawrate=100.0, hover=0.6)')
    ax.legend()

    # Grafik görüntüsünü bellekte sakla
    fig.canvas.draw()
    frame_buffer = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
    frame_buffer = frame_buffer.reshape(fig.canvas.get_width_height()[::-1] + (3,))
    frames.append(frame_buffer)

# Animasyon oluştur
ani = FuncAnimation(fig, update, frames=len(vio_data), interval=100, repeat=False)  # 100 ms arayla güncelleme

# Canlı grafiği göster
plt.show()

# Canlı gösterim kapatıldıktan sonra GIF kaydet
print("GIF kaydediliyor...")
images = [Image.fromarray(frame) for frame in frames]
images[0].save("drone_trajectory8.gif", save_all=True, append_images=images[1:], duration=100, loop=0)
print("GIF kaydedildi: drone_trajectory8.gif")
