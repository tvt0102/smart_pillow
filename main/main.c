#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "FileServer.h"

// Pin TX
#define PIN_TX1 GPIO_NUM_26
#define PIN_TX2 GPIO_NUM_27

// Thời gian us
#define START_LOW   80
#define START_HIGH  24
#define BIT1_LOW    32
#define BIT1_HIGH   16
#define BIT0_LOW    16
#define BIT0_HIGH   16

// Mode
#define MODE_DIEU_KHIEN   0b1101000110100001
#define MODE_TEST_LED     0b1111100010100001
#define MODE_DIM_LED      0b1100010110100001
#define DIM_FIXED         0b0000111100000001
#define DIM_EXTRA8        0b01100100

#define FRAME_INTERVAL_MS 8 // ms

#define PIN_SENSOR GPIO_NUM_14

volatile bool receiving_sequence = false;
volatile uint64_t seq_start_time_us = 0;
volatile uint64_t last_frame_time_us = 0;

// ====== Biến toàn cục điều khiển ======
bool testMode = false;
uint32_t led_level = 0xFFFFFFF0;
uint8_t pwm_value = 100;
uint8_t led_steps = 0;
uint16_t step_delay_ms = 500; // 0.5 giây
int frameCountInPhase = 0;
uint8_t effect_steps = 0;
volatile bool run_effect_flag = false;
volatile bool update_steps_flag = false;

// GPTimer handle
static gptimer_handle_t gptimer = NULL;
static volatile bool timer_done = false;

SemaphoreHandle_t led_mutex = NULL;

// ====== GPTimer callback ======
static bool IRAM_ATTR timer_callback(gptimer_handle_t timer,
                                     const gptimer_alarm_event_data_t *edata,
                                     void *user_data) {
    timer_done = true;
    return false;
}

// ====== Khởi tạo GPTimer ======
static void timer_init_us(void) {
    gptimer_config_t config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000 // 1 tick = 1 µs  
    };
    gptimer_new_timer(&config, &gptimer);

    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_callback
    };
    gptimer_register_event_callbacks(gptimer, &cbs, NULL);

    gptimer_enable(gptimer);
}

// ====== Delay us không busy-wait ======
static inline void IRAM_ATTR wait_us(uint32_t us) {
    timer_done = false;
    gptimer_set_raw_count(gptimer, 0);

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = us,
        .flags.auto_reload_on_alarm = false
    };
    gptimer_set_alarm_action(gptimer, &alarm_config);

    gptimer_start(gptimer);
    while (!timer_done) {
        taskYIELD(); // Nhường CPU
    }
    gptimer_stop(gptimer);
}

// ====== GPIO control ======
static void IRAM_ATTR sensor_isr_handler(void* arg) {
    uint64_t now_us = esp_timer_get_time();

    if (!receiving_sequence) {
        receiving_sequence = true;
        seq_start_time_us = now_us;
    }
    last_frame_time_us = now_us;
}


static inline void set_gpio_both(int level) {
    gpio_set_level(PIN_TX1, level);
    gpio_set_level(PIN_TX2, level);
}

static void send_bit(bool bitVal) {
    set_gpio_both(0);
    wait_us(bitVal ? BIT1_LOW : BIT0_LOW);
    set_gpio_both(1);
    wait_us(bitVal ? BIT1_HIGH : BIT0_HIGH);
}

static void send_start(void) {
    set_gpio_both(0);
    wait_us(START_LOW);
    set_gpio_both(1);
    wait_us(START_HIGH);
}

static void send_frame(uint16_t mode16, uint32_t val32,
                       uint16_t extra16, uint8_t pwm, uint8_t extra8, bool hasExtra) {
    send_start();
    for (int i = 15; i >= 0; i--) send_bit((mode16 >> i) & 1);
    if (hasExtra) {
        for (int i = 15; i >= 0; i--) send_bit((extra16 >> i) & 1);
        for (int i = 7; i >= 0; i--) send_bit((pwm >> i) & 1);
        for (int i = 7; i >= 0; i--) send_bit((extra8 >> i) & 1);
    } else {
        for (int i = 31; i >= 0; i--) send_bit((val32 >> i) & 1);
    }
}

// ====== WiFi AP ======
static void wifi_init_ap(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t ap_config = {
        .ap = {
            .ssid = "ESP32_LED",
            .ssid_len = 0,
            .channel = 1,
            .password = "66668888",
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    if (strlen((char *)ap_config.ap.password) == 0) {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &ap_config);
    esp_wifi_start();
}
// ====== Task tính LED từ số bậc ======
void effect_task(void *arg) {
    while (1) {
        if (run_effect_flag) {
            led_level = 0;
            for (int i = 0; i < effect_steps; i++) {
                led_level |= (1UL << i);
                vTaskDelay(step_delay_ms);
            }
            run_effect_flag = false;
            ESP_LOGI("TX TASK","Mode hiệu ứng: Led level: %ld", led_level);
        }
        else if (update_steps_flag){
            led_level = 0; // Tắt hết trước
            for (int i = 0; i < led_steps; i++) {
                led_level |= (1UL << i); // Bật thêm 1 bậc
            }
            ESP_LOGI("TX TASK","Mode thường: Led level: %ld", led_level);
            update_steps_flag = false;   
        }
        else {
            vTaskDelay(5);
        }
    }
}
// ====== Task gửi dữ liệu ======
static void tx_task(void *arg) {
    while (1) {
        if (!testMode) {
            send_frame(MODE_DIEU_KHIEN, led_level, 0, 0, 0, false);
        } else {
            ESP_LOGI("TX TASK","Test mode: %d", testMode);
            ESP_LOGI("TX TASK","PWM: %d", pwm_value);
            if (frameCountInPhase % 2 == 0) {
                send_frame(MODE_DIM_LED, 0, DIM_FIXED, pwm_value, DIM_EXTRA8, true);
            } else {
                send_frame(MODE_TEST_LED, led_level, 0, 0, 0, false);
            }
            frameCountInPhase++;
            if (frameCountInPhase >= 16) {
                testMode = false;
                frameCountInPhase = 0;
            }
        }
        vTaskDelay(FRAME_INTERVAL_MS);
    }
}

static void sensor_task(void *arg) {
    while (1) {
        uint64_t now_us = esp_timer_get_time();

        if (receiving_sequence && (now_us - last_frame_time_us) > 10000) {
            receiving_sequence = false;

            float seq_duration_ms = (last_frame_time_us - seq_start_time_us) / 1000.0f;

            if (seq_duration_ms >= 20 && seq_duration_ms <= 45) {
                printf("Phát hiện cảm biến 1 (%.1f ms)\n", seq_duration_ms);
            } else if (seq_duration_ms >= 70 && seq_duration_ms <= 90) {
                printf("Phát hiện cảm biến 2 (%.1f ms)\n", seq_duration_ms);
            } else if (seq_duration_ms > 100) {
                printf("Phát hiện cả 2 cảm biến (%.1f ms)\n", seq_duration_ms);
            } else {
                printf("Không xác định (%.1f ms)\n", seq_duration_ms);
            }
        }

        vTaskDelay(1);
    }
}

static void sensor_init(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE, // Start pulse HIGH
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PIN_SENSOR),
        .pull_up_en = 0,
        .pull_down_en = 0
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_SENSOR, sensor_isr_handler, NULL);

    xTaskCreatePinnedToCore(sensor_task, "SensorTask", 2048, NULL, 5, NULL, 0);
}

// ====== MAIN ======
void app_main(void) {
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << PIN_TX1) | (1ULL << PIN_TX2),
    };

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    led_mutex = xSemaphoreCreateMutex();

    gpio_config(&io_conf);
    set_gpio_both(1);

    wifi_init_ap();
    timer_init_us();
    sensor_init();

    xTaskCreatePinnedToCore(tx_task, "TX_Task", 4096, NULL, 8, NULL, 1);
    xTaskCreatePinnedToCore(effect_task, "Effect_Task", 2048, NULL, 5, NULL, 1);
    start_file_server();
    
    while (1) {
        vTaskDelay(100);
    }
}
