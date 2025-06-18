/**
 * @file main.c
 * @brief Main file for SmartPillow
 * @version 2.0
 * @copyright Copyright (c) 2025
 */

 /*------------------------------------ INCLUDE LIBRARY ------------------------------------ */

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include <esp_vfs_fat.h>
#include <esp_log.h>
#include <esp_vfs.h>
#include <freertos/semphr.h>
#include <inttypes.h>

#include "mqtt_client.h"
#include "driver/gpio.h"
#include "cJSON.h"
#include "esp_sntp.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/i2c.h"

#include "../component/DS3231/ds3231.h"
#include "../component/Time/DS3231Time.h"
#include "../component/WebServer/FileServer.h" 
#include "../component/INMP441/inmp441.h"
#include "../component/FileManager/sdcard.h"
#include "max30102.h"
#include "../component/wavelet/wavelet.h"

/*------------------------------------ DEFINE ------------------------------------ */
static const char *TAG = "SMART PILLOW";
//WIFI
#define WIFI_SSID "Nghe House 2"
#define WIFI_PASSWORD "@ngoinhavuive"

//MQTT
#define MQTT_BROKER_URL  "mqtt://192.168.1.9:1883"

// RTC
#define CONFIG_RTC_I2C_PORT 0
#define CONFIG_RTC_PIN_NUM_SDA 26
#define CONFIG_RTC_PIN_NUM_SCL 27

//Max30102
#define SAMPLE_LEN 500
#define SAMPLE_RATE 200
#define powerLed      UINT8_C(0x1F) // Cường độ led, tiêu thụ 6.4mA
#define sampleAverage 4
#define ledMode       2
#define sampleRate    100 // Tần số lấy mẫu cao thì kích thước BUFFER_SIZE cũng phải thay đổi để có thời gian thuật toán xử lý các mẫu
#define pulseWidth    411 // Xung càng rộng, dải thu được càng nhiều (18 bit)
#define adcRange      16384 // 14 bit ADC tiêu thụ 65.2pA mỗi LSB
#define I2C_SDA_GPIO  21
#define I2C_SCL_GPIO  22
#define I2C_PORT      I2C_NUM_0
extern const double rdb4_low_pass_filter[FILTER_SIZE];

//Buffer de luu tru du lieu doc duoc tu buffer DMA
//Chuyen doi tu byte DMA sang so luong mau cua moi buffer 
static int16_t buffer16[DMA_BUFFER_SIZE / sizeof(int32_t) * 3 / 2] = {0}; //288 samples (576 bytes) de luu duoc 3 bytes sau khi dich cua buffer32
static int32_t buffer32[DMA_BUFFER_SIZE / sizeof(int32_t)] = {0}; //192 samples (768 bytes)

//Tao kenh rx
i2s_chan_handle_t rx_channel = NULL; 

TaskHandle_t readMAXTask_handle = NULL;
TaskHandle_t readINMTask_handle = NULL;
TaskHandle_t controlPillow_handle = NULL;

i2c_dev_t ds3231_device;

// namefile save into sd card
char nameFilePCG[15];
char nameFilePPG[15];

esp_mqtt_client_handle_t client;

enum functionControl {
    top,
    left,
    right,
    bottom
};
int indexControl = -1;
int LOW = 0;

int s_retry_num = 0;
int status;  // variable to save status message from MQTT to control pillow
char str[30];  // variable to save message
bool check = false;

void controlPillow(void* parameter){
    ESP_LOGI(__func__, "Status to control pillow: %d\n", status);
    //gpio_pad_select_gpio(2);
    //config maybom1 - 2// may2 -4 // may3-16 // thoatkhi1 - 17 / thoatkhi2 - 3 / thoatkhi3 - 1

    while(true){
        if(status == 0){
            break;
        }
        indexControl++;
        indexControl = indexControl % 4;
        //ESP_LOGI(__func__, "Top: %d %d\n", indexControl, top);
        if(indexControl == top){
            ESP_LOGI(__func__, "Status to control top\n");
            gpio_set_level(2, status); // control may bom 1
            gpio_set_level(4, status); // control may bom 2
            gpio_set_level(16, status); // control may bom 3
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            gpio_set_level(2, LOW); // stop may bom 1
            gpio_set_level(4, LOW); // stop may bom 2
            vTaskDelay(10000 / portTICK_PERIOD_MS);
            gpio_set_level(16, LOW); // stop may bom 3
            break;
                
        } 
        if(indexControl == left){
            ESP_LOGI(__func__, "Status to control left\n");
            gpio_set_level(2, status); // bom 1
            gpio_set_level(3, status); // xa van 2
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            gpio_set_level(2, LOW); 
            gpio_set_level(3, LOW); 
            break;
                
        } 
        if(indexControl == right){
            ESP_LOGI(__func__, "Status to control right\n");
            gpio_set_level(17, status); // xa van 1
            gpio_set_level(4, status); // bom 2
            vTaskDelay(10000 / portTICK_PERIOD_MS);
            gpio_set_level(17, LOW); 
            gpio_set_level(4, LOW); 
            break;
                
        } 
        if(indexControl == bottom){
            ESP_LOGI(__func__, "Status to control bottom\n");
            gpio_set_level(3, status); // xa van 2
            gpio_set_level(1, status); // xa van 3
            vTaskDelay(15000 / portTICK_PERIOD_MS);
            gpio_set_level(3, LOW); 
            gpio_set_level(1, LOW); 
            break;
                
        } 
        
    }
    
    // while(left == indexControl && status == 1){
    //     ESP_LOGI(__func__, "Status to control left\n");
    //     TickType_t startTime = xTaskGetTickCount();
    //     gpio_set_level(2, status); //  bom 1
    //     gpio_set_level(8, status); //  open van 2
    //     TickType_t endTime = xTaskGetTickCount();
            
    //     if(endTime - startTime >= 5000){
    //         gpio_set_level(2, 0); // stop may bom 1
    //         gpio_set_level(8, 0); // close van 2
    //         break;
    //     }
    // }
    // while(right == indexControl && status == 1){
    //     ESP_LOGI(__func__, "Status to control right\n");
    //     TickType_t startTime = xTaskGetTickCount();
    //     gpio_set_level(5, status); // open van 1
    //     gpio_set_level(4, status); // bom 2
    //     TickType_t endTime = xTaskGetTickCount();
            
    //     if(endTime - startTime >= 10000){
    //         gpio_set_level(5, 0); // close van 1
    //         gpio_set_level(4, 0); // stop may bom 2
    //         break;
    //     }
    // }
    // while(bottom == indexControl && status == 1){
    //     ESP_LOGI(__func__, "Status to control bottom\n");
    //     TickType_t startTime = xTaskGetTickCount();
    //     gpio_set_level(5, status); // open 1
    //     gpio_set_level(18, status); // open 2
    //     gpio_set_level(19, status); // open 3
    //     TickType_t endTime = xTaskGetTickCount();
            
    //     if(endTime - startTime >= 15000){
    //         gpio_set_level(5, status); // close 1
    //         gpio_set_level(18, status); // close 2
    //         gpio_set_level(19, status); // close 3
    //     }
    
    // }
    
    vTaskDelete(NULL);    
    
}


/*------------------------------------ MQTT ------------------------------------ */

static void log_error_if_nonzero(const char * message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(__func__, "Last error %s: 0x%x", message, error_code);
    }
}

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    client = event->client;
    int msg_id;
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(__func__, "MQTT_EVENT_CONNECTED");
            msg_id = esp_mqtt_client_subscribe(client, "pillow/control", 1);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(__func__, "MQTT_EVENT_DISCONNECTED");
            break;

        // case MQTT_EVENT_SUBSCRIBED:
        //     ESP_LOGI(__func__, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        //     msg_id = esp_mqtt_client_subscribe(client, "pillow/control", 1);
        //     // ESP_LOGI(__func__, "sent publish successful, msg_id=%d", msg_id);
        //     break;
        // case MQTT_EVENT_UNSUBSCRIBED:
        //     ESP_LOGI(__func__, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        //     break;
        // case MQTT_EVENT_PUBLISHED:
        //     ESP_LOGI(__func__, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        //     break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(__func__, "MQTT_EVENT_DATA");
            int index = 0;
            check = true;
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            sprintf(str, "%.*s\r\n", event->data_len, event->data);
            //printf("DATA=%.*s\r\n", event->data_len, event->data);
            while(str[index] != ':'){
                index++;
            }
            status = str[index+1] - '0';
            printf("Status: %s\n", str);
            xTaskCreatePinnedToCore(controlPillow, "controlPillow", 1024 * 5,NULL,15,&controlPillow_handle, 0);
            //controlPillow(status);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(__func__, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
                log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
                log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
                ESP_LOGI(__func__, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

            }
            break;
        default:
            ESP_LOGI(__func__, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(__func__, "Event dispatched from event loop base=%s, event_id=%ld", base, event_id);
    mqtt_event_handler_cb(event_data);
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URL,
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}

/*------------------------------------ WIFI ------------------------------------ */
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < 10) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(__func__, "retry to connect to the AP");
        }
        ESP_LOGI(__func__,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {      
        //mqtt_app_start(); // initinal mqtt
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(__func__, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        start_file_server(base_path);
        s_retry_num = 0;
    }
}

void WIFI_initSTA(void)
{

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            /* Authmode threshold resets to WPA2 ass default if password matches WPA2 standards (pasword len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(__func__, "wifi_init_sta finished.");
}

/*------------------------------------ MQTT ------------------------------------ */
void publish_message(const char* topic, const char*nameFile) {
    // Tạo đối tượng JSON
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        ESP_LOGE(__func__, "Failed to create JSON object");
        return;
    }

    // Thêm các trường vào JSON
    cJSON_AddStringToObject(root, "namefile", nameFile);
    
   
    // Chuyển đổi đối tượng JSON thành chuỗi
    char *json_string = cJSON_Print(root);
    if (json_string == NULL) {
        ESP_LOGE(__func__, "Failed to print JSON string");
        cJSON_Delete(root);
        return;
    }

    // Publish thông điệp JSON
    int msg_id = esp_mqtt_client_publish(client, topic, json_string, 0, 0, 0);
    ESP_LOGI(__func__, "Sent publish successful, msg_id=%d", msg_id);

    // Giải phóng bộ nhớ
    cJSON_Delete(root);
    free(json_string);
}



/*-------------------------------------TASKS--------------------------------------------*/
/**
 * @brief Read data from MAX30102 and send to ring buffer
 * 
 * @param pvParameters 
 */
SemaphoreHandle_t sdcard_write_mutex;
esp_err_t max30102_configure(i2c_dev_t *dev, struct max30102_record *record){
    // Khởi tạo mô tả thiết bị I2C cho MAX30102
    memset(dev, 0, sizeof(i2c_dev_t));
    ESP_ERROR_CHECK(max30102_initDesc(dev, I2C_PORT, I2C_SDA_GPIO, I2C_SCL_GPIO));
    
    if(max30102_readPartID(dev) == ESP_OK) {
      ESP_LOGI(__func__, "Found MAX30102 at address 0x%02x on port %d!", dev->addr, dev->port);
    }
    else {
      ESP_LOGE(__func__, "Not found MAX30102 at address 0x%02x on port %d", dev->addr, dev->port);
      return ESP_FAIL;
    }

    // Khởi tạo các thông số hoạt động của MAX30102
    ESP_ERROR_CHECK(max30102_init(powerLed, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange, record, dev));
    max30102_clearFIFO(dev);
    ESP_LOGI(__func__, "MAX30102 configured successfully.");
    return ESP_OK;
}
void read_max30102_task(void* parameter)
{
    
    i2c_dev_t dev;
    struct max30102_record record;
    if(max30102_configure(&dev, &record) != ESP_OK){
        ESP_LOGE(pcTaskGetName(NULL), "Khoi tao cam bien khong thanh cong, loi I2C...");
    }else ESP_LOGI(pcTaskGetName(NULL), "Khoi tao cam bien thanh cong !");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    char temp_buffer[1450] = {0};
    size_t offset = 0;
    int sampleCount = 0;
    unsigned long red;
    unsigned long ir;
    
    struct tm timeTemp = { 0 };
    ds3231_get_time(&ds3231_device, &timeTemp);
    sprintf(nameFilePPG,"%s_%d_%d_%d", "PPG",timeTemp.tm_hour,timeTemp.tm_min,timeTemp.tm_sec);
    ESP_LOGI(__func__, "Get data MAX30102 start file %s\n", nameFilePPG);

    while (1)
    {
        //vTaskDelay(10);
        uint16_t number_of_newSample = max30102_check(&record, &dev); //Check the sensor, read up to 3 samples
        //ESP_LOGI(__func__, "Number of new samples: %d", number_of_newSample);
        while (max30102_available(&record)) //do we have new data?
        {
            red = max30102_getFIFORed(&record);
            ir = max30102_getFIFOIR(&record);
            int len = snprintf(temp_buffer+offset, sizeof(temp_buffer)-offset, "%lu,%lu\n", red, ir);
            if ((offset+len) < (int)(sizeof(temp_buffer))){
                offset += len;
                sampleCount++;
            } // tránh tràn
            else {
                ESP_LOGE(__func__, "Buffer overflow");
            }
            if (sampleCount >= 100) // Check if sampleCount is greater than 500
            { 
                if (xSemaphoreTake(sdcard_write_mutex, portMAX_DELAY) == pdTRUE) { // Lấy semaphore
                    esp_err_t err = sdcard_writeDataToFile_noArgument(nameFilePPG, temp_buffer);
                    xSemaphoreGive(sdcard_write_mutex); // Nhả semaphore
                    if (err != ESP_OK) {
                        ESP_LOGE(__func__, "Ghi dữ liệu MAX30102 vào SD card thất bại: %s", esp_err_to_name(err));
                    }
                    else {
                        ESP_LOGI(__func__, "Ghi dữ liệu MAX30102 vào SD card thành công.");
                    }
                }
                else {
                    ESP_LOGE(__func__, "Không thể lấy semaphore ghi SD card cho MAX30102.");
                }
                memset(temp_buffer, 0, sizeof(temp_buffer));
                offset = 0;
                vTaskDelay(pdMS_TO_TICKS(1));
                ESP_LOGI(__func__, "Writing file %s is done", nameFilePPG);
                publish_message("message/nameFilePPG", nameFilePPG);
                ds3231_get_time(&ds3231_device, &timeTemp);
                sprintf(nameFilePPG,"%s_%d_%d_%d", "PPG",timeTemp.tm_hour,timeTemp.tm_min,timeTemp.tm_sec); 
                sampleCount = 0;
                ESP_LOGI(__func__, "Get data MAX30102 start file %s\n", nameFilePPG);
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            max30102_nextSample(&record); //We're finished with this sample so move to next sample
        }
        //ESP_LOGI(__func__, "End of MAX30102 while loop");
    }
    //ESP_LOGI(__func__, "End of MAX30102 task");
    //vTaskDelete(NULL);
}


static void initialize_nvs(void)
{
    esp_err_t error = nvs_flash_init();
    if (error == ESP_ERR_NVS_NO_FREE_PAGES || error == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        error = nvs_flash_init();
    }
    ESP_ERROR_CHECK(error);
}

/**
 * @brief Read data from INMP441 and send to ring buffer
 * 
 * @param pvParameters 
 */

void readINMP441Task(void* parameter) {
    i2s_install(&rx_channel); // Cấu hình kênh I2S sử dụng API mới
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(__func__, "Bắt đầu đọc dữ liệu từ INMP441...");

    //TickType_t startTime = xTaskGetTickCount();
    size_t bytesRead;
    int sampleCount = 0;
    char temp_buffer[4096] = {0}; // đủ để lưu 585 mẫu
    size_t offset = 0;

    struct tm timeTemp = {0};
    ds3231_get_time(&ds3231_device, &timeTemp); // Lấy thời gian thực
    memset(nameFilePCG, 0, sizeof(nameFilePCG));
    sprintf(nameFilePCG, "PCG_%02d_%02d_%02d", timeTemp.tm_hour, timeTemp.tm_min, timeTemp.tm_sec);
    ESP_LOGI(__func__, "Get data INMP441 start file: %s", nameFilePCG);
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10)); // tránh watchdog reset
        // Đọc dữ liệu từ microphone
        esp_err_t ret = i2s_channel_read(rx_channel, &buffer32, sizeof(buffer32), &bytesRead, 100);
        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGE(__func__, "Timeout khi đọc dữ liệu: %s", esp_err_to_name(ret));
            continue;
        } else if (ret != ESP_OK) {
            ESP_LOGE(__func__, "Lỗi đọc dữ liệu: %s", esp_err_to_name(ret));
            break;
        }
        int samplesRead = bytesRead / sizeof(int32_t);
        //ESP_LOGI(__func__, "Số mẫu dữ liệu INMP441: %d", samplesRead);
        for (int i = 0; i < samplesRead; i++) {
            sampleCount++;
            int16_t sample = (int16_t)(buffer32[i] >> 8); // Lấy 16-bit có ý nghĩa từ 24-bit gốc
            buffer16[i] = sample; 
            int len = snprintf(temp_buffer + offset, sizeof(temp_buffer) - offset, "%d\n",buffer16[i]);
            offset += len;
        }
        if(offset >= sizeof(temp_buffer)) {
            ESP_LOGI(__func__, "Buffer full, writing data to file...");
        }
        if (sampleCount >=550) {
            if (xSemaphoreTake(sdcard_write_mutex, portMAX_DELAY) == pdTRUE) { // Lấy semaphore
                esp_err_t err = sdcard_writeDataToFile_noArgument(nameFilePCG, temp_buffer);
                xSemaphoreGive(sdcard_write_mutex); // Nhả semaphore
                if (err != ESP_OK) {
                    ESP_LOGE(__func__, "Ghi dữ liệu INMP441 vào SD card thất bại: %s", esp_err_to_name(err));
                }
                else {
                    ESP_LOGI(__func__, "Ghi dữ liệu INMP441 vào SD card thành công.");
                }
            }
            else {
                ESP_LOGE(__func__, "Không thể lấy semaphore ghi SD card cho INMP441.");
            }
            publish_message("message/nameFilePCG", nameFilePCG);
            ds3231_get_time(&ds3231_device, &timeTemp);
            sprintf(nameFilePCG, "PCG_%02d_%02d_%02d", timeTemp.tm_hour, timeTemp.tm_min, timeTemp.tm_sec);
            ESP_LOGI(__func__, "Get data INMP441 start file: %s", nameFilePCG);
            sampleCount = 0;
            memset(temp_buffer, 0, sizeof(temp_buffer));
            offset = 0;
            vTaskDelay(pdMS_TO_TICKS(1000));
            //startTime = currentTime;
            
        } // tránh tràn
    }
}

void sntp_init_func()
{
    ESP_LOGI(__func__, "Initializing SNTP.");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
    sntp_init();
}

esp_err_t sntp_setTime(struct tm *timeInfo, time_t *timeNow)
{
    for (size_t i = 0; (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET); i++)
    {
        ESP_LOGI(__func__, "Waiting for system time to be set...");
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    time(timeNow);
    localtime_r(timeNow, timeInfo);

    char timeString[64];

    // Set timezone to VietNam Standard Time
    setenv("TZ", "GMT-07", 1);
    tzset();
    localtime_r(timeNow, timeInfo);
    strftime(timeString, sizeof(timeString), "%c", timeInfo);
    ESP_LOGI(__func__, "The current date/time in Viet Nam is: %s ", timeString);
    return ESP_OK;
}

/*--------------------------------------MAIN_APP-------------------------------------------*/
void app_main(void)
{
    // Initialize SPI Bus
    
    ESP_LOGI(__func__, "Initialize SD card with SPI interface.");
    esp_vfs_fat_mount_config_t mount_config_t = MOUNT_CONFIG_DEFAULT();
    spi_bus_config_t spi_bus_config_t = SPI_BUS_CONFIG_DEFAULT();
    sdmmc_host_t host_t = SDSPI_HOST_DEFAULT();
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = CONFIG_PIN_NUM_CS;
    slot_config.host_id = host_t.slot;

    sdmmc_card_t SDCARD;
    ESP_ERROR_CHECK(sdcard_initialize(&mount_config_t, &SDCARD, &host_t, &spi_bus_config_t, &slot_config));
    sdcard_write_mutex = xSemaphoreCreateMutex();
    if (sdcard_write_mutex == NULL) {
        ESP_LOGE(__func__, "Failed to create SD card write mutex");
        return;
    } else {
        ESP_LOGI(__func__, "SD card write mutex created OK");
    }

    ESP_LOGI(__func__, "Initialize nvs partition.");
    initialize_nvs();
    // Wait 1 second for memory initialization
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    WIFI_initSTA();
    //initinal mqtt
    mqtt_app_start();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI(__func__, "Set up output control pillow\n");
    gpio_set_direction(2, GPIO_MODE_OUTPUT);    // bom 1
    gpio_set_direction(4, GPIO_MODE_OUTPUT);    // bom 2
    gpio_set_direction(16, GPIO_MODE_OUTPUT);   // bom 3

    gpio_set_direction(17, GPIO_MODE_OUTPUT);    // xa 1
    gpio_set_direction(3, GPIO_MODE_OUTPUT);    // xa 2
    gpio_set_direction(1, GPIO_MODE_OUTPUT);   // xa 3
    
    //update time use sntp
    // time_t timeNow = 0;
    // struct tm timeInfo = { 0 };
    // sntp_init_func();
    // sntp_setTime(&timeInfo, & timeNow);
    // mktime(&timeInfo);

    ESP_LOGI(__func__, "Initialize DS3231 module(I2C/Wire%d).", CONFIG_RTC_I2C_PORT);
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2cdev_init());
    memset(&ds3231_device, 0, sizeof(i2c_dev_t));
    ESP_ERROR_CHECK_WITHOUT_ABORT(ds3231_initialize(&ds3231_device, CONFIG_RTC_I2C_PORT, CONFIG_RTC_PIN_NUM_SDA, CONFIG_RTC_PIN_NUM_SCL));
    // timeInfo.tm_sec += 2;
    // timeInfo.tm_mon += 1;
    // timeInfo.tm_year = 2024;

    //ds3231_set_time(&ds3231_device, &timeInfo);

    // Create tasks
    xTaskCreatePinnedToCore(read_max30102_task, "read_max30102_task", 1024 * 25,NULL, 20, &readMAXTask_handle, 0);
    xTaskCreatePinnedToCore(readINMP441Task, "readINM411", 1024 * 25, NULL, 19, &readINMTask_handle, 1);  // ?? Make max30102 task and inm task have equal priority can make polling cycle of max3012 shorter ??  

    //xTaskCreatePinnedToCore(sendDataToServer, "sendDataToServer", 1024 * 10,NULL,  10, &sendDataToServer_handle, 0);
    //xTaskCreatePinnedToCore(listenFromMQTT, "listenFromMQTT", 1024 * 3,NULL,  5, &listenFromMQTT_handle, 0);
}
