#include <stdio.h>
#include <stdlib.h>

int main() {
    FILE *input_file = fopen("dataMAX.txt", "r");  // Mở file .txt đầu vào
    if (!input_file) {
        perror("Không thể mở file .txt đầu vào");
        return EXIT_FAILURE;
    }

    FILE *output_file = fopen("data.csv", "w");  // Mở file .csv đầu ra
    if (!output_file) {
        perror("Không thể mở file .csv đầu ra");
        fclose(input_file);
        return EXIT_FAILURE;
    }

    char line[256];  // Dùng để đọc từng dòng từ file .txt
    while (fgets(line, sizeof(line), input_file)) {
        double val1, val2;
        // Đọc từng giá trị trong dòng
        if (sscanf(line, "%lf,%lf", &val1, &val2) == 2) {
            // Ghi các giá trị vào file .csv
            fprintf(output_file, "%.6lf,%.6lf\n", val1, val2);
        }
    }

    fclose(input_file);  // Đóng file .txt
    fclose(output_file);  // Đóng file .csv

    printf("Chuyển đổi từ .txt sang .csv thành công!\n");
    return 0;
}
