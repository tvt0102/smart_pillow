#include "DS3231Time.h"

//#define CHECK_ARG(ARG) do { if (!ARG) return ESP_ERR_INVALID_ARG; } while (0)

// %d-%d-%d
static const char *timeFormat2 = "%d-%d-%d";

static const int month[12] = {31,28,31,30,31,30,31,31,30,31,30,31};

 int currentDay;

esp_err_t ds3231_initialize(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    esp_err_t error;
    error = ds3231_init_desc(dev, port, sda_gpio, scl_gpio);
    if (error != ESP_OK)
    {
        ESP_LOGE(__func__, "DS3231 module initialize fail.");
        return error;
    }
    ESP_LOGI(__func__, "DS3231 module initialize success.");
    struct tm currentTime;
    ds3231_get_time(dev, &currentTime);
    currentDay = currentTime.tm_yday;
    return error;
}

esp_err_t ds3231_convertTimeToString(i2c_dev_t *dev, char* timeString, const unsigned int lenghtString)
{
    struct tm time;
    ds3231_get_time(dev, &time);
    memset(timeString, 0, lenghtString);
    int lenght = 0;
    lenght = sprintf(timeString, timeFormat2, time.tm_hour, time.tm_min, time.tm_sec, time.tm_year, time.tm_mon, time.tm_mday);
    if (lenght)
    {
        ESP_LOGI(__func__, "Convert time to string success.");
        return ESP_OK;
    }

    ESP_LOGE(__func__, "Convert time to string fail.");
    return ESP_FAIL;

}

esp_err_t ds3231_setTime(i2c_dev_t *dev, struct tm *time)
{
    return ds3231_set_time(dev, time);
}

esp_err_t ds3231_getEpochTime(i2c_dev_t *dev, uint32_t *epochTime)
{
    struct tm currentTime = {0};
    *epochTime = 0;
    esp_err_t err_code = ESP_OK;
    err_code = ds3231_get_time(dev, &currentTime);
    if (err_code != ESP_OK)
    {
        ESP_LOGE(__func__, "DS3231 get UnixTime fail(%s).", esp_err_to_name(err_code));
        return ESP_FAIL;
    }

    for (size_t i = 0; i < (currentTime.tm_mon - 1); i++)
    {
        currentTime.tm_yday += month[i];
        if (i == 1) currentTime.tm_yday += ((currentTime.tm_year % 4) == 0);
    }
    currentTime.tm_yday += (currentTime.tm_mday - 1);

    *epochTime += SECONDS_FROM_1970_TO_2023;
    *epochTime += ((currentTime.tm_year - 2023) * SECONDS_PER_YEAR);
    *epochTime += (currentTime.tm_yday * SECONDS_PER_DAY);
    *epochTime += (currentTime.tm_hour * SECONDS_PER_HOUR);
    *epochTime += (currentTime.tm_min  * SECONDS_PER_MIN);
    *epochTime += currentTime.tm_sec;
    ESP_LOGI(__func__, "DS3231 get EpochTime success.");
    
    return ESP_OK;
}

bool ds3231_isNewDay(i2c_dev_t *dev)
{
    struct tm currentTime;
    ds3231_get_time(dev, &currentTime);
    return (currentDay == currentTime.tm_yday);
}
