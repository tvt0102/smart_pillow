#ifndef WAVELET_H
#define WAVELET_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <stddef.h>
#define FILTER_SIZE 8  // Độ dài bộ lọc
#define SR 500 // Tần số lấy mẫu

void symmetric_padding(const double *input, int length, int pad_size, double *output) ;
void convolve(const double *signal, int signal_len, const double *filter, int filter_len, double *result);
void filter_and_downsample(const double *signal, int signal_len, const double *filter, int filter_len, double *result, int *result_len);
void wavelet_transform(const double *signal, int signal_len, double *cA, int *cA_len, double *cD, int *cD_len) ;
int read_signal_block_from_csv(FILE *file, double *signal, int block_size);
void write_signal_to_csv(const char *filename, const double *signal, int length, const char *mode);
void upsample(const double *signal, int signal_len, double *upsampled_signal, int upsample_factor);
int find_peaks(const double *signal, int signal_len, int *peaks, int max_peaks, double threshold);
double calculate_heart_rate(const int *peaks, int peak_count, double sample_rate);
double calculate_spo2(const double *red_signal, const double *ir_signal, int signal_len);

#endif // WAVELET_H