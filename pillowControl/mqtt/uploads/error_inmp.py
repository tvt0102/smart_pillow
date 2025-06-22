
file_path = 'D:/ESP/Espressif/frameworks/pillowControl/mqtt/uploads/dataINMP.txt'
import sys

# Đọc dữ liệu từ file input.txt
def read_input_file(file_path):
    encodings = ['latin1', 'windows-1252', 'utf-8']  # Thử latin1 đầu tiên
    for encoding in encodings:
        try:
            with open(file_path, 'r', encoding=encoding) as file:
                lines = [line.rstrip('\n') for line in file]
            print(f"Read file successfully with encoding: {encoding}")
            print(f"Total lines: {len(lines)}")
            return lines
        except UnicodeDecodeError as e:
            print(f"Error reading with encoding {encoding}: {str(e)}")
            continue
        except FileNotFoundError:
            print(f"File not found: {file_path}")
            return []
        except Exception as e:
            print(f"Other error reading file: {str(e)}")
            return []
    print("Could not read file with any encoding.")
    return []

# Hàm kiểm tra xem một chuỗi có phải số nguyên không
def is_integer(s):
    s = s.strip()
    if not s:  # Dòng rỗng là lỗi
        return False
    try:
        int(s)
        return True
    except ValueError:
        return False

# Đọc dữ liệu
lines = read_input_file(file_path)

if lines:
    # Đếm số lỗi và lưu chi tiết
    error_details = [(i, line) for i, line in enumerate(lines) if not is_integer(line)]
    errors = len(error_details)
    error_positions = [i + 1 for i, _ in error_details]  # Đếm từ 1

    # Tính khoảng cách giữa các lỗi liên tiếp
    error_distances = [error_details[i+1][0] - error_details[i][0] - 1 for i in range(len(error_details)-1)]

    # Tính khoảng cách lỗi trung bình
    average_distance = sum(error_distances) / len(error_distances) if error_distances else 0

    # In kết quả
    print(f"Number of errors: {errors}")
    print(f"Error line positions (1-based): {error_positions}")
    print(f"Error line contents: {[f'Line {i+1}: {line}' for i, line in error_details]}")
    print(f"Distances between errors: {error_distances}")
    print(f"Average error distance: {average_distance:.2f}")

else:
    print("No data to process. Please check input.txt.")