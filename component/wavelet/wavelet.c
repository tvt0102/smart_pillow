#include "wavelet.h"
#include <math.h>
#include <string.h>

#define FILTER_SIZE 8  // Độ dài bộ lọc
#define SR 500 // Tần số lấy mẫu

const double rdb4_low_pass_filter[FILTER_SIZE] = {0.23037781330885, 0.7148465705525415, 0.6308807679295904, -0.02798376941698385, -0.18703481171888114, 0.030841381835986965, 0.032883011666982945, -0.010597401784997278};
const double db4_low_pass_filter[FILTER_SIZE] = {-0.010597401784997278, 0.032883011666982945, 0.030841381835986965, -0.18703481171888114, -0.02798376941698385, 0.6308807679295904, 0.7148465705525415, 0.23037781330885523};
const double db4_high_pass_filter[FILTER_SIZE] = {-0.23037781330885523, 0.7148465705525415, -0.6308807679295904, -0.02798376941698385, 0.18703481171888114, 0.030841381835986965, -0.032883011666982945, -0.010597401784997278};
const double rdb4_high_pass_filter[FILTER_SIZE] = {-0.010597401784997278, -0.032883011666982945, 0.030841381835986965, 0.18703481171888114, -0.02798376941698385, -0.6308807679295904, 0.7148465705525415, -0.23037781330885523};
// Bộ đệm dối xứng
void symmetric_padding(const double *input, int length, int pad_size, double *output) {
    int padded_length = length + 2 * pad_size;

    // Copy phần đầu đối xứng
    for (int i = 0; i < pad_size; i++) {
        output[i] = input[pad_size - i];  // Phản chiếu phần đầu
    }

    // Copy dữ liệu gốc
    for (int i = 0; i < length; i++) {
        output[i + pad_size] = input[i];
    }

    // Copy phần cuối đối xứng
    for (int i = 0; i < pad_size; i++) {
        output[length + pad_size + i] = input[length - 1 - i];  // Phản chiếu phần cuối
    }
}

void convolve(const double *signal, int signal_len, const double *filter, int filter_len, double *result) {

    int output_len = signal_len + filter_len - 1;
    for (int i = 0; i < output_len; i++) {
        result[i] = 0.0;  // Kh?i t?o giá tr? 0
        for (int j = 0; j < filter_len; j++) {
            if (i - j >= 0 && i - j < signal_len) {  // Ki?m tra gi?i h?n
                result[i] += signal[i - j] * filter[j];
            }
        }
    }
}

// Thực hiện tích chập và downsampling
void filter_and_downsample(const double *signal, int signal_len, const double *filter, int filter_len, double *result, int *result_len) {
    int pad_size = filter_len / 2;  // Padding bằng nửa kích thước bộ lọc
    int padded_len = signal_len + 2 * pad_size;
    double *padded_signal = (double *)malloc(padded_len * sizeof(double));

    // Áp dụng symmetric padding
    symmetric_padding(signal, signal_len, pad_size, padded_signal);

    // Kích thước tín hiệu sau tích chập
    int filtered_len = padded_len + filter_len - 1;
    double *filtered_signal = (double *)malloc(filtered_len * sizeof(double));

    // Thực hiện tích chập với tín hiệu đã padding
    convolve(padded_signal, padded_len, filter, filter_len, filtered_signal);

    // Giảm kích thước (downsampling)
    int index = 0;
    for (int i = 0; i < filtered_len; i += 2) {  // Lấy giá trị ở vị trí chẵn
        result[index++] = filtered_signal[i];
    }

    *result_len = index;  // Cập nhật độ dài kết quả

    // Giải phóng bộ nhớ
    free(padded_signal);
    free(filtered_signal);
}

void wavelet_transform(const double *signal, int signal_len, double *cA, int *cA_len, double *cD, int *cD_len) {
    // Lọc tín hiệu bằng bộ lọc Low_pass
    filter_and_downsample(signal, signal_len, db4_low_pass_filter, FILTER_SIZE, cA, cA_len);

    // Lọc tín hiệu bằng bộ lọc High_pass
    filter_and_downsample(signal, signal_len, db4_high_pass_filter, FILTER_SIZE, cD, cD_len);
}

// Hàm đọc một khối dữ liệu từ file CSV
int read_signal_block_from_csv(FILE *file, double *signal, int block_size) {
    int count = 0;
    while (count < block_size && fscanf(file, "%lf", &signal[count]) == 1) {
        count++;
    }
    return count;  // Trả về số lượng mẫu đã đọc
}

// Hàm ghi kết quả vào file CSV
void write_signal_to_csv(const char *filename, const double *signal, int length, const char *mode) {
    FILE *file = fopen(filename, mode);
    if (!file) {
        perror("Không thể mở file để ghi");
        exit(EXIT_FAILURE);
    }

    for (int i = 0; i < length; i++) {
        fprintf(file, "%.6lf\n", signal[i]);
    }

    fclose(file);
}

void upsample(const double *signal, int signal_len, double *upsampled_signal, int upsample_factor) {
    int index = 0;
    for (int i = 0; i < signal_len; i++) {
        upsampled_signal[index++] = signal[i];
        for (int j = 1; j < upsample_factor; j++) {
            upsampled_signal[index++] = 0.0;  // Chèn giá trị 0
        }
    }
}

// Hàm tìm các đỉnh tín hiệu
int find_peaks(const double *signal, int signal_len, int *peaks, int max_peaks, double threshold) {
    int peak_count = 0;
    int i=1;
    while (i<signal_len-1) {
        if (signal[i] > threshold && signal[i] > signal[i - 1] && signal[i] > signal[i + 1]) {
            if (peak_count < max_peaks) {
                peaks[peak_count++] = i;
                i+=100;
            } else {
                break;
            }
        }
        else i++;
    }
    return peak_count;
}

// Hàm tính nhịp tim từ các đỉnh
// Công thức tính HR
/*
    HR = 60 / RRmean
    RRmean = (1/(tần số lấy mẫu)) * (khoảng cách giữa 2 đỉnh)

*/
double calculate_heart_rate(const int *peaks, int peak_count, double sample_rate) {
    if (peak_count < 2) return 0.0;

    double rr_intervals_sum = 0.0;
    for (int i = 1; i < peak_count; i++) {
      //  printf("%f\n", (peaks[i] - peaks[i - 1]) / sample_rate);
        rr_intervals_sum += (peaks[i] - peaks[i - 1]) / sample_rate;
    }
    double rr_mean = rr_intervals_sum / (peak_count - 1);
    return 60.0 / rr_mean; // Nhịp tim tính bằng bpm
}

// Hàm tính SpO2 (giả định tín hiệu Red và IR đã được tách)

/*
    Công thức tính SpO2: 
        SpO2 = 110 - 25*R


        với R= (ACred / DCred) / (ACir / DCir)

*/
double calculate_spo2(const double *red_signal, const double *ir_signal, int signal_len) {
    double ac_red = 0.0, ac_ir = 0.0, dc_red = 0.0, dc_ir = 0.0;

    for (int i = 0; i < signal_len; i++) {
        dc_red += red_signal[i];
        dc_ir += ir_signal[i];
        ac_red += fabs(red_signal[i] - dc_red / signal_len);
        ac_ir += fabs(ir_signal[i] - dc_ir / signal_len);
    }

    double ratio = (ac_red / dc_red) / (ac_ir / dc_ir);
    double spo2 = 110 - 25 * ratio; // Công thức tham khảo, cần điều chỉnh tùy tín hiệu
    return spo2;
}