import numpy as np
import matplotlib.pyplot as plt

# Đọc dữ liệu từ file .txt
def read_data(file_path):
    # Đọc dữ liệu từ file
    data = np.loadtxt(file_path, delimiter=',')
    red_signal = data[:, 0]  # Lấy cột RED
    ir_signal = data[:, 1]  # Lấy cột IR
    return ir_signal, red_signal

# Vẽ tín hiệu IR và RED
def plot_signals(ir_signal, red_signal):
    plt.figure(figsize=(10, 6))
    
    # Vẽ tín hiệu IR
    plt.plot(ir_signal, label='IR Signal', color='red')
    
    # Vẽ tín hiệu RED
    plt.plot(red_signal, label='RED Signal', color='blue')
    
    # Thêm tiêu đề và nhãn
    plt.title("IR and RED Signals")
    plt.xlabel("Sample Index")
    plt.ylabel("Signal Amplitude")
    
    # Thêm legend
    plt.legend()
    
    # Hiển thị đồ thị
    plt.grid(True)
    plt.show()

# Đường dẫn tới file dữ liệu
file_path = 'PPG_22_30_8.txt'  # Đổi tên file nếu cần

# Đọc dữ liệu từ file và vẽ đồ thị
ir_signal, red_signal = read_data(file_path)
plot_signals(ir_signal, red_signal)
