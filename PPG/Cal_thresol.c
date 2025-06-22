#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define MAX_SIZE 1000  // Định nghĩa kích thước tối đa của tín hiệu

// Hàm đọc dữ liệu từ file .txt
int read_data_from_txt(const char *filename, double signal[]) {
    FILE *file = fopen(filename, "r");
    if (file == NULL) {
        printf("Không thể mở file.\n");
        return -1;
    }

    int i = 0;
    double red, ir;
    while (fscanf(file, "%lf,%lf", &red, &ir) != EOF) {
        signal[i] = ir;  // Hoặc dùng red tùy thuộc vào dữ liệu bạn cần
        i++;
        if (i >= MAX_SIZE) {
            break;
        }
    }

    fclose(file);
    return i;  // Trả về số lượng mẫu đã đọc
}

// Hàm tính trung bình
double calculate_mean(double signal[], int size) {
    double sum = 0.0;
    for (int i = 0; i < size; i++) {
        sum += signal[i];
    }
    return sum / size;
}

// Hàm tính độ lệch chuẩn
double calculate_std_dev(double signal[], int size, double mean) {
    double sum_sq = 0.0;
    for (int i = 0; i < size; i++) {
        sum_sq += (signal[i] - mean) * (signal[i] - mean);
    }
    return sqrt(sum_sq / size);
}

// Hàm tính ngưỡng threshold
double calculate_threshold(double mean, double std_dev) {
    return mean + std_dev;  // Ngưỡng threshold = mean + 2 * std_dev
}

int main() {
    double signal[MAX_SIZE];
    const char *filename = "dataMAX.txt";  // Đường dẫn tới file .txt của bạn

    int size = read_data_from_txt(filename, signal);  // Đọc dữ liệu từ file
    if (size == -1) {
        return 1;
    }

    double mean = calculate_mean(signal, size);  // Tính trung bình
    double std_dev = calculate_std_dev(signal, size, mean);  // Tính độ lệch chuẩn
    double threshold = calculate_threshold(mean, std_dev);  // Tính ngưỡng threshold

    printf("Trung bình: %.2f\n", mean);
    printf("Độ lệch chuẩn: %.2f\n", std_dev);
    printf("Ngưỡng threshold: %.2f\n", threshold);

    return 0;
}
