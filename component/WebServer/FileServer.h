#ifndef __FILESERVER_H__
#define __FILESERVER_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_http_server.h"

// Các biến và hàm LED từ main.c
extern bool testMode;
extern uint8_t led_steps;
extern uint8_t pwm_value;
extern void update_led_from_steps(uint8_t steps);
extern uint16_t step_delay_ms;
extern uint8_t effect_steps;
extern void update_led_from_effect_steps(uint8_t steps);
extern int frameCountInPhase;
extern volatile bool run_effect_flag;
extern volatile bool update_steps_flag;

// Hàm khởi tạo HTTP server
esp_err_t start_file_server(void);

#endif
