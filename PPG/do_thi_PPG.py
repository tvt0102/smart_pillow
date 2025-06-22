import matplotlib.pyplot as plt

# Đọc dữ liệu từ file txt
filename = "D:/20242/DATN/PPG/dataMAX.txt"  # Đổi đường dẫn tới file .txt của bạn
ir_data = []
red_data = []

# Mở file và đọc dữ liệu
with open(filename, "r") as file:
    for line in file:
        values = line.strip().split(",")  # Tách dữ liệu IR và RED
        ir_data.append(int(values[0]))  # Giá trị IR
        red_data.append(int(values[1]))  # Giá trị RED

# Vẽ đồ thị IR
plt.figure(figsize=(10, 5))
plt.plot(ir_data, label="IR")
plt.xlabel('Mẫu')
plt.ylabel('Giá trị IR')
plt.title('Đồ thị tín hiệu IR')
plt.legend()
plt.grid(True)
plt.show()

# Vẽ đồ thị RED
plt.figure(figsize=(10, 5))
plt.plot(red_data, label="RED", color='r')
plt.xlabel('Mẫu')
plt.ylabel('Giá trị RED')
plt.title('Đồ thị tín hiệu RED')
plt.legend()
plt.grid(True)
plt.show()
