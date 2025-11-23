import pandas as pd
import matplotlib.pyplot as plt

# CSV dosyasını oku
#df = pd.read_csv('test_emre_pid_pose_data_outputmpc.csv')
df = pd.read_csv('test_emre.csv')
df = pd.read_csv('test2hover.csv')

# Zaman damgalarını saniye cinsine çevir (nano saniyeden)
df['%time'] = df['%time'] / 1e9
df['field.header.stamp'] = df['field.header.stamp'] / 1e9

# Görselleştirme
plt.figure(figsize=(14, 7))

# Pozisyon grafikleri
plt.subplot(2, 1, 1)
plt.plot(df['%time'], df['field.values0'], label='Pozisyon X')
plt.plot(df['%time'], df['field.values1'], label='Pozisyon Y')
plt.plot(df['%time'], df['field.values2'], label='Pozisyon Z')
#plt.axhline(y=1, color='r', linestyle='--', label='Referans')
plt.title('Z Pozisyon Hareketi')
plt.xlabel('Zaman (s)')
plt.ylabel('Pozisyon (m)')
plt.legend()

# Oryantasyon grafikleri
plt.subplot(2, 1, 2)
plt.plot(df['%time'], df['field.values4'], label='Sapma-X')
plt.plot(df['%time'], df['field.values5'], label='Sapma-Y')
plt.plot(df['%time'], df['field.values6'], label='Sapma-Z')
plt.plot(df['%time'], df['field.values3'], label='Sapma-Yaw')
plt.title('Dönme Hareketi')
plt.xlabel('Zaman (s)')
plt.ylabel('Oryantasyon (quaternion)')
plt.legend()

plt.tight_layout()
plt.show()
