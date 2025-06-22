def analyze_data_errors(file_path):
    error_lines = []
    line_numbers = []
    
    with open(file_path, 'r', encoding='utf-8', errors='ignore') as file:
        lines = file.readlines()
    
    for i, line in enumerate(lines):
        line = line.strip()
        if not line:
            continue
        try:
            parts = line.split(",")
            if len(parts) != 2:
                raise ValueError("Không có đúng 2 phần tử")
            int(parts[0])
            int(parts[1])
        except Exception:
            error_lines.append(line)
            line_numbers.append(i)

    # Tính khoảng cách giữa các lỗi
    error_distances = [
        j - i for i, j in zip(line_numbers[:-1], line_numbers[1:])
    ]
    
    avg_distance = (
        sum(error_distances) / len(error_distances)
        if error_distances else 0
    )
    print(f"So dong: {len(lines)}")
    print(f"So dong loi: {len(error_lines)}")
    print(f"Khoang cach loi: {error_distances}")
    print(f"Trung binh khoang cach: {avg_distance:.2f}")

    return {
        "so dong": len(lines),
        "so loi": len(error_lines),
        "khoang_cach": error_distances,
        "trung_binh": avg_distance
    }

# Gọi hàm với đường dẫn tới file
result = analyze_data_errors("D:/ESP/Espressif/frameworks/pillowControl/mqtt/uploads/dataMAX.txt")
