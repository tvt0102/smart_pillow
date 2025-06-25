import math
import matplotlib.pyplot as plt
import pywt
import scipy.signal as sp_signal  # Đảm bảo dùng scipy.signal đúng cách
import sys
import numpy as np
import librosa
from pydub import AudioSegment
import scipy.io.wavfile as wav
import paho.mqtt.client as mqtt
import json
import time

# Cho phép in tiếng Việt
sys.stdout.reconfigure(encoding='utf-8')

FILTER_SIZE = 8
SR = 8000  # Tần số lấy mẫu

# Cấu hình MQTT Broker
BROKER = "192.168.1.18"
PORT = 1883
TOPIC = "pillow/control"

# Hàm gửi status (1 = bất thường, 0 = bình thường)
def send_status(status_value):
    client = mqtt.Client()

    client.connect(BROKER, PORT, 60)

    # Tạo bản tin JSON
    data = {
        "status": status_value
    }

    # Chuyển thành chuỗi JSON và gửi
    payload = json.dumps(data)
    client.publish(TOPIC, payload)
    print(f"Đã gửi status = {status_value} đến ESP32")

    client.disconnect()

# Bộ lọc db4
db4_low = [-0.010597401784997278, 0.032883011666982945, 0.030841381835986965,
           -0.18703481171888114, -0.02798376941698385, 0.6308807679295904,
           0.7148465705525415, 0.23037781330885523]

rdb4_low_pass_filter = [0.23037781330885, 0.7148465705525415, 0.6308807679295904,
            -0.02798376941698385, -0.18703481171888114, 0.030841381835986965,
            0.032883011666982945, -0.010597401784997278]

# Hàm đọc dữ liệu từ file .txt
def read_txt_file(filename):
    signal = []
    with open(filename, 'r') as f:
        for line in f:
            # Đọc từng giá trị mẫu tín hiệu từ file .txt
            signal.append(float(line.strip()))
    return np.array(signal)
# Hàm đọc dữ liệu từ file .bin (2 byte cho mỗi mẫu)
def read_bin_file(filename):
    signal = []
    with open(filename, 'rb') as f:  # Mở file nhị phân
        while byte := f.read(2):  # Đọc mỗi 2 byte (1 mẫu)
            signal.append(int.from_bytes(byte, byteorder='little', signed=True))  # Chuyển 2 byte thành 1 mẫu int16
    return np.array(signal)

# Hàm chuyển đổi tín hiệu thành file .wav
def save_as_wav(signal, filename, sample_rate=SR):
    # Đảm bảo tín hiệu nằm trong khoảng từ -1 đến 1 để phù hợp với định dạng WAV
    signal = np.int16(signal / np.max(np.abs(signal)) * 32767)
    wav.write(filename, sample_rate, signal)  # Lưu tín hiệu vào file .wav

# Hàm chuyển đổi tín hiệu thành file .mp3
def save_as_mp3(signal, filename, sample_rate=SR):
    # Chuyển tín hiệu thành mảng numpy, đảm bảo độ sâu bit 16-bit
    signal = np.int16(signal / np.max(np.abs(signal)) * 32767)
    
    # Lưu tín hiệu vào file .wav trước khi chuyển sang mp3
    wav_filename = "temp_signal.wav"
    save_as_wav(signal, wav_filename, sample_rate)
    
    # Chuyển file .wav thành .mp3 bằng pydub
    sound = AudioSegment.from_wav(wav_filename)
    sound.export(filename, format="mp3")
    
    # Xóa file tạm thời sau khi chuyển đổi
    import os
    os.remove(wav_filename)


# Hàm đọc dữ liệu từ file CSV
# Hàm đọc dữ liệu từ file WAV hoặc MP3
def read_audio_file(filename):
    # Đọc tín hiệu từ file audio (WAV, MP3, etc.)
    signal, sr = librosa.load(filename, sr=SR)
    return signal, sr

# Hàm tính trung bình
def calculate_mean(signal):
    return sum(signal) / len(signal)

# Hàm tính độ lệch chuẩn
def calculate_std_dev(signal, mean):
    return math.sqrt(sum((x - mean) ** 2 for x in signal) / len(signal))

# Hàm tính ngưỡng threshold
def calculate_threshold(mean, std_dev):
    return mean + 2*std_dev  # Ngưỡng threshold = mean + std_dev

# Hàm tìm đỉnh tín hiệu
def find_peaks(signal, threshold, min_distance = 8000):
    peaks = []
    i = 1
    while i < len(signal) - 1:
        if signal[i] > threshold and signal[i] > signal[i-1] and signal[i] > signal[i+1]:
            peaks.append(i)
            i += min_distance  # Giảm nhiễu và tránh đỉnh liên tiếp
        else:
            i += 1
    return peaks

# Bộ lọc bandpass
def bandpass_filter(signal, lowcut, highcut, sample_rate, order=4):
    nyquist = 0.5 * sample_rate  # Tính tần số Nyquist
    if lowcut <= 0 or highcut >= nyquist:
        raise ValueError(f"Tần số cắt không hợp lệ: {lowcut} - {highcut} Hz. Tần số cắt phải nằm trong khoảng (0, Nyquist).")
    
    if lowcut >= highcut:
        raise ValueError(f"Tần số cắt thấp phải nhỏ hơn tần số cắt cao: {lowcut} - {highcut} Hz.")
    
    # Chuẩn hóa tần số cắt so với tần số Nyquist
    low = lowcut / nyquist
    high = highcut / nyquist
    
    # Áp dụng bộ lọc Butterworth
    b, a = sp_signal.butter(order, [low, high], btype='band')
    filtered_signal = sp_signal.filtfilt(b, a, signal)  # Dùng scipy.signal.filtfilt để áp dụng bộ lọc
    return filtered_signal

# Hàm vẽ tín hiệu với các đỉnh
def plot_signal_with_peaks(signal, peaks):
    plt.figure(figsize=(10, 4))
    plt.plot(signal, label='Tín hiệu phục hồi', color='orange')
    plt.plot(peaks, [signal[i] for i in peaks], 'ro', label='Đỉnh')
    plt.title("Tín hiệu âm thanh sau khôi phục và tìm đỉnh")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

# Hàm vẽ tín hiệu trước và sau khi biến đổi wavelet
def plot_before_after_wavelet(signal, transformed_signal):
    plt.figure(figsize=(10, 6))
    plt.subplot(2, 1, 1)
    plt.plot(signal, label="Tín hiệu âm thanh sau khi lọc")
    plt.title("Tín hiệu âm thanh sau khi lọc")
    plt.legend()
    
    plt.subplot(2, 1, 2)
    plt.plot(transformed_signal, label="Tín hiệu sau khi biến đổi wavelet", color='orange')
    plt.title("Tín hiệu sau khi biến đổi wavelet")
    plt.legend()
    
    plt.tight_layout()
    plt.show()
# Hàm vẽ tín hiệu trước và sau khi lọc
def plot_before_after_filter(signal, filtered_signal):
    plt.figure(figsize=(10, 6))
    plt.subplot(2, 1, 1)
    plt.plot(signal, label="Tín hiệu âm thanh gốc")
    plt.title("Tín hiệu âm thanh gốc")
    plt.legend()
    
    plt.subplot(2, 1, 2)
    plt.plot(filtered_signal, label="Tín hiệu sau khi lọc", color='orange')
    plt.title("Tín hiệu sau khi lọc")
    plt.legend()
    
    plt.tight_layout()
    plt.show()

# Hàm biến đổi wavelet sử dụng thư viện pywt
def wavelet_transform(signal, wavelet='db4', level=3):
    coeffs = pywt.wavedec(signal, wavelet, level=level)
    return coeffs  # Trả về các thành phần approximation (cA) và detail (cD)

# Hàm phục hồi tín hiệu
def restore_signal(cA):
    # Tăng mẫu tín hiệu (upsample) để khôi phục tín hiệu
    def upsample(signal, factor):
        return np.repeat(signal, factor)
    
    # Upsample để nhân mẫu
    ucA = upsample(cA, 2)  # Nhân mẫu gấp đôi (2x)
    k = len(cA) * 2
    ucA1 = np.convolve(ucA, rdb4_low_pass_filter, mode='same')  # Chập với bộ lọc rdb4_low_pass_filter
    k = k * 2
    ucA1 = upsample(ucA1, 2)  # Upsample thêm một lần nữa
    ucA1 = np.convolve(ucA1, rdb4_low_pass_filter, mode='same')  # Chập lại
    k = k * 2
    ucA1 = upsample(ucA1, 2)  # Upsample lần cuối
    ucA1 = np.convolve(ucA1, rdb4_low_pass_filter, mode='same')  # Chập lại

    return ucA1  # Trả về tín hiệu khôi phục

# Hàm tính nhịp thở từ các đỉnh
def calculate_breath_rate(peaks, sample_rate):
    if len(peaks) < 2:
        return 0.0

    breath_intervals = []  # Khởi tạo danh sách để lưu trữ thời gian giữa các lần thở

    for i in range(1, len(peaks)):
        interval = (peaks[i] - peaks[i-1]) / sample_rate + 0.8   # Tính khoảng cách thời gian giữa hai lần thở
        print(f"Thời gian giữa 2 lần thở {i} và {i+1}: {interval:.4f} giây")
        breath_intervals.append(interval)  # Lưu lại khoảng thời gian giữa các lần thở

    breath_mean = sum(breath_intervals) / len(breath_intervals)  # Tính thời gian trung bình giữa các lần thở
    breath_rate = 60.0 / breath_mean  # Nhịp thở (thở/phút)
    return breath_rate

# Hàm vẽ các cấp độ wavelet
def plot_wavelet_levels(coeffs):
    level = len(coeffs) - 1  # Số cấp độ
    plt.figure(figsize=(10, 12))

    for i in range(level):
        cA = coeffs[i]
        plt.subplot(level + 1, 1, i + 1)
        plt.plot(cA, label=f'cA Level {i + 1}', color='blue' if i == 0 else 'orange')
        plt.title(f'Approximation Coefficients (cA) Level {i + 1}')
        plt.legend()

    plt.tight_layout()
    plt.show()
# === MAIN ===
# Đọc tín hiệu từ file .txt
#D:/ESP/Espressif/frameworks/pillowControl/mqtt/uploads/
bin_filename = "dataINMP.bin"  # Đặt tên file .bin chứa dữ liệu tín hiệu
bin_signal = read_bin_file(bin_filename)
    
# Lưu tín hiệu thành file .wav
save_as_wav(bin_signal, "dataINMP.wav", sample_rate=SR)
filename = "dataINMP.wav" 
signal, sample_rate = read_audio_file(filename)

# Lọc tín hiệu IR với bộ lọc bandpass (0.5 Hz - 50 Hz)
filtered_signal = bandpass_filter(signal, lowcut=20, highcut=1000, sample_rate=SR)
plot_before_after_filter(signal, filtered_signal)
# Phân tích 3 cấp bằng wavelet
coeffs = pywt.wavedec(filtered_signal, 'db4', level=3)

# Vẽ các cấp độ wavelet
plot_wavelet_levels(coeffs)

# Khôi phục lại tín hiệu (giữ lại cả approximation và detail)
cA = coeffs[0]
cD = coeffs[1:]
restored_signal = restore_signal(cA)

# Tính threshold từ tín hiệu phục hồi
mean = calculate_mean(restored_signal)
std_dev = calculate_std_dev(restored_signal, mean)
threshold = calculate_threshold(mean, std_dev)

# Tìm đỉnh & nhịp tim
peaks = find_peaks(restored_signal, threshold)
breath_rate = calculate_breath_rate(peaks, SR)

# In kết quả
print(f"Ngưỡng threshold: {threshold:.2f}")
print(f"Số lượng đỉnh: {len(peaks)}")
print(f"Nhịp thở: {breath_rate:.2f} nhịp/phút")
for i, p in enumerate(peaks):
    print(f"Đỉnh {i+1} tại vị trí {p}")

# Vẽ tín hiệu với các đỉnh
plot_signal_with_peaks(restored_signal, peaks)

# Vẽ tín hiệu trước và sau khi biến đổi wavelet
plot_before_after_wavelet(filtered_signal, restored_signal)

   
