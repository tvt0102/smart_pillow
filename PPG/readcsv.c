#include <stdio.h>
#include <stdlib.h>

int main() {
    FILE *file = fopen("data.csv", "r");  // Mở file CSV
    if (!file) {
        perror("Không thể mở file CSV");
        return EXIT_FAILURE;
    }

    double value;  // Biến để lưu trữ giá trị đọc được từ file
    printf("Giá trị trong file CSV:\n");

    // Đọc từng giá trị trong file và in ra màn hình
    while (fscanf(file, "%lf", &value) == 1) {
        printf("%.6lf\n", value);  // In ra giá trị đọc được
    }

    fclose(file);  // Đóng file sau khi đọc xong
    return 0;
}
