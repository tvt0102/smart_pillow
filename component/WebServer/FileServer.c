#include "FileServer.h"

static const char *TAG = "FileServer";

// HTML đã nhúng bằng EMBED_FILES
extern const uint8_t upload_script_html_start[] asm("_binary_upload_script_html_start");
extern const uint8_t upload_script_html_end[]   asm("_binary_upload_script_html_end");

// Handler trả về HTML
static esp_err_t index_handler(httpd_req_t *req)
{
    size_t html_size = upload_script_html_end - upload_script_html_start;
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char *)upload_script_html_start, html_size);
    return ESP_OK;
}
// Hàm decode URL (chuyển %3A -> :, %26 -> &, ...)
static void url_decode(char *dst, const char *src, int dstsize) {
    char a, b;
    while (*src && dstsize > 1) {
        if ((*src == '%') &&
            ((a = src[1]) && (b = src[2])) &&
            (isxdigit(a) && isxdigit(b))) {
            a = (a >= 'A') ? (a & 0xdf) - 'A' + 10 : (a - '0');
            b = (b >= 'A') ? (b & 0xdf) - 'A' + 10 : (b - '0');
            *dst++ = (char)(16 * a + b);
            src += 3;
        } else if (*src == '+') {
            *dst++ = ' ';
            src++;
        } else {
            *dst++ = *src++;
        }
        dstsize--;
    }
    *dst = '\0';
} 

// Handler nhận lệnh điều khiển
static esp_err_t cmd_handler(httpd_req_t *req)
{
    char query[128];
    char buf[128];
    char decoded[128];

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        if (httpd_query_key_value(query, "data", buf, sizeof(buf)) == ESP_OK) {
            url_decode(decoded, buf, sizeof(decoded));
            ESP_LOGI(TAG, "CMD: %s", decoded);
            // Tách nhiều lệnh bằng '&'
            char *token = strtok(decoded, "&");
            while (token != NULL) {
                if (strncmp(token, "PWM:", 4) == 0) {
                    pwm_value = atoi(token + 4);
                    if (pwm_value > 100) pwm_value = 100;
                    // Khi có PWM thì tự bật test mode
                    testMode = true;
                    frameCountInPhase = 0;
                }
                else if (strncmp(token, "STEP:", 5) == 0) {
                    uint8_t steps = atoi(token + 5);
                    if (steps > 32) steps = 32;
                    led_steps = steps;
                    update_steps_flag = true;
                }
                else if (strncmp(token, "EFFECT:", 7) == 0) {
                    int steps, delay;
                    sscanf(token + 7, "%d,%d", &steps, &delay);
                    if (steps > 32) steps = 32;
                    effect_steps = steps;
                    step_delay_ms = delay;
                    run_effect_flag = true;
                }   
                token = strtok(NULL, "&");
            }

            httpd_resp_sendstr(req, "OK");
            return ESP_OK;
        }
    }

    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing data param");
    return ESP_FAIL;
}

// Khởi tạo HTTP server
esp_err_t start_file_server(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    ESP_LOGI(TAG, "Starting HTTP Server on port %d", config.server_port);
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start server");
        return ESP_FAIL;
    }

    // URI /
    httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_handler
    };
    httpd_register_uri_handler(server, &index_uri);

    // URI /cmd
    httpd_uri_t cmd_uri = {
        .uri = "/cmd",
        .method = HTTP_GET,
        .handler = cmd_handler
    };
    httpd_register_uri_handler(server, &cmd_uri);

    return ESP_OK;
}
