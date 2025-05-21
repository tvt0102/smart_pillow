/**
 * @note ADC cua inmp441 la 24-bit nhung van chuyen tu 32-bit(esp32 de xu ly hon) 
 * ve 24-bit cua inmp roi moi ep ve 16-bit chu khong lay truc tiep 24-bit
 * @author Luong Huu Phuc
 */
#ifndef INMP441_H
#define INMP441_H

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/gpio_types.h"
#include "driver/gpio.h"
#include <esp_log.h>

//Thu vien I2S
#include <driver/i2s_std.h> //API moi <- Dung cai nay
#include <driver/i2s_types_legacy.h>
#include "driver/i2s_types.h"
#include "driver/i2s_common.h"

//Pin cau hinh INMP441 (I2S Mode)
#define I2S_NUM I2S_NUM_0
#define I2S_SCK_PIN 32
#define I2S_WS_PIN 25
#define I2S_SD_PIN 33
#define SAMPLE_RATE 4000 //Tan so lay mau 4000Hz

/**
 * @note Sample rate cang cao thi dmaLen cung cang cao de tranh mat mau
 * @param dmaLen Tang len neu du lieu bi mat mau, CPU bi ngat nhieu ma khong quan tam den latency
 * ->Cang lon thi buffer_durations cang lau->Cang it mat mau->CPU xu ly it viec hon->Tre lai cao hon
 * @param dmaDesc So bo dac ta DMA, moi bo co the luu tru so byte = dmaLen 
 */
#define dmaDesc 6 //Bo dac ta DMA
#define dmaLen 128 //So bytes cua moi buffer
#define DMA_BUFFER_SIZE (dmaLen * dmaDesc) //So bytes cua buffer DMA cung cap cho = 768　       

//Cau hinh i2s std (I2S_std)
void i2s_install(i2s_chan_handle_t **rx_channel);
#endif