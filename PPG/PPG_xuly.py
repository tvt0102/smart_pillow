import math
import matplotlib.pyplot as plt
import pywt
import scipy.signal as sp_signal  # Đảm bảo dùng scipy.signal đúng cách
import sys
import numpy as np

# Cho phép in tiếng Việt
sys.stdout.reconfigure(encoding='utf-8')

FILTER_SIZE = 8
SR = 200  # Tần số lấy mẫu

# Bộ lọc db4
db4_low = [-0.010597401784997278, 0.032883011666982945, 0.030841381835986965,
           -0.18703481171888114, -0.02798376941698385, 0.6308807679295904,
           0.7148465705525415, 0.23037781330885523]

rdb4_low_pass_filter = [0.23037781330885, 0.7148465705525415, 0.6308807679295904,
            -0.02798376941698385, -0.18703481171888114, 0.030841381835986965,
            0.032883011666982945, -0.010597401784997278]

# Hàm đọc dữ liệu từ file CSV
def read_csv(filename):
    red, ir = [], []
    with open(filename, 'r') as f:
        for line in f:
            if ',' in line:
                r, i = line.strip().split(',')
                red.append(float(r))
                ir.append(float(i))
    return red, ir

# Hàm tính trung bình
def calculate_mean(signal):
    return sum(signal) / len(signal)

# Hàm tính độ lệch chuẩn
def calculate_std_dev(signal, mean):
    return math.sqrt(sum((x - mean) ** 2 for x in signal) / len(signal))

# Hàm tính ngưỡng threshold
def calculate_threshold(mean, std_dev):
    return mean + std_dev  # Ngưỡng threshold = mean + std_dev

# Hàm tìm đỉnh tín hiệu
def find_peaks(signal, threshold, min_distance=40):
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
    plt.plot(signal, label='Tín hiệu phục hồi')
    plt.plot(peaks, [signal[i] for i in peaks], 'ro', label='Đỉnh')
    plt.title("IR sau khôi phục và tìm đỉnh")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

# Hàm vẽ tín hiệu trước và sau khi biến đổi wavelet
def plot_before_after_wavelet(signal, transformed_signal):
    plt.figure(figsize=(10, 6))
    plt.subplot(2, 1, 1)
    plt.plot(signal, label="Tín hiệu IR gốc")
    plt.title("Tín hiệu IR gốc")
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
    plt.plot(signal, label="Tín hiệu IR gốc")
    plt.title("Tín hiệu IR gốc")
    plt.legend()
    
    plt.subplot(2, 1, 2)
    plt.plot(filtered_signal, label="Tín hiệu sau khi lọc", color='orange')
    plt.title("Tín hiệu sau khi loc")
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

# Hàm tính nhịp tim từ các đỉnh
def calculate_heart_rate(peaks, sample_rate):
    if len(peaks) < 2:
        return 0.0
    rr_intervals = [(peaks[i] - peaks[i-1]) / sample_rate for i in range(1, len(peaks))]
    rr_mean = sum(rr_intervals) / len(rr_intervals)
    return 60.0 / rr_mean  # Nhịp tim = 60 / RR_mean
# Hàm tính SpO2 từ tín hiệu IR và RED
def calculate_spo2(red_signal, ir_signal):
    ac_red = sum(abs(red_signal[i] - calculate_mean(red_signal)) for i in range(len(red_signal)))
    ac_ir = sum(abs(ir_signal[i] - calculate_mean(ir_signal)) for i in range(len(ir_signal)))
    dc_red = calculate_mean(red_signal)
    dc_ir = calculate_mean(ir_signal)

    ratio = (ac_red / dc_red) / (ac_ir / dc_ir)
    spo2 = 110 - 25 * ratio
    return spo2

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
red_signal, ir_signal = read_csv("data.csv")
signal_len = len(ir_signal)

# Lọc tín hiệu IR với bộ lọc bandpass (0.5 Hz - 50 Hz)
filtered_ir_signal = bandpass_filter(ir_signal, lowcut=0.5, highcut=50, sample_rate=SR)
plot_before_after_filter(ir_signal, filtered_ir_signal)
# Phân tích 3 cấp bằng wavelet
coeffs = pywt.wavedec(filtered_ir_signal, 'db4', level=3)

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
hr = calculate_heart_rate(peaks, SR)

# In kết quả
print(f"Ngưỡng threshold: {threshold:.2f}")
print(f"Số lượng đỉnh: {len(peaks)}")
print(f"Nhịp tim: {hr:.2f} BPM")
for i, p in enumerate(peaks):
    print(f"Đỉnh {i+1} tại vị trí {p}")

# Tính SpO2 từ tín hiệu IR và RED
spo2 = calculate_spo2(red_signal, ir_signal)
print(f"SpO2: {spo2:.2f}%")

# Vẽ tín hiệu với các đỉnh
plot_signal_with_peaks(restored_signal, peaks)

# Vẽ tín hiệu trước và sau khi biến đổi wavelet
plot_before_after_wavelet(filtered_ir_signal, restored_signal)
