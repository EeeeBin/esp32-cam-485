#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"

#include "string.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "esp_camera.h"

#include "rs485.h"
#include "uart_frame.h"

/* Define ------------------------------------------------------------------- */
//M5STACK_CAM PIN Map
#define CAM_PIN_PWDN    -1 //power down is not used
#define CAM_PIN_RESET   15 //software reset will be performed
#define CAM_PIN_XCLK    27
#define CAM_PIN_SIOD    22
#define CAM_PIN_SIOC    23

#define CAM_PIN_D7      19
#define CAM_PIN_D6      36
#define CAM_PIN_D5      18
#define CAM_PIN_D4      39
#define CAM_PIN_D3      5
#define CAM_PIN_D2      34
#define CAM_PIN_D1      35
#define CAM_PIN_D0      32

#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    26
#define CAM_PIN_PCLK    21

#define CAM_XCLK_FREQ   20000000

/* Static var --------------------------------------------------------------- */
static const char* TAG = "camera";

/* Static fun --------------------------------------------------------------- */

extern void led_brightness(int duty);

static camera_config_t camera_config = {
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    //XCLK 20MHz or 10MHz
    .xclk_freq_hz = CAM_XCLK_FREQ,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,//YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_UXGA,//QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 9, //0-63 lower number means higher quality
    .fb_count = 1 //if more than one, i2s runs in continuous mode. Use only with JPEG
};

// if receive uart frame(20ms idle), will get rx_buffer
static void uart_rx_task(void *arg) {
    rx_buffer_t *rx_buffer = NULL;
    for(;;) {
        if(uart_buffer_quenue != 0){
            if(xQueueReceive(uart_buffer_quenue, &rx_buffer, (TickType_t)portMAX_DELAY)){       
                rs485_deal(rx_buffer->buf, rx_buffer->buf_size);
                free(rx_buffer->buf);
                free(rx_buffer);
            }
        }
    }
    vTaskDelete(NULL);
}

// cam addr in rs485.c defind
// rs485 recvive will callback
void rs485_cmd_callback(uint8_t cmd, int8_t *data, uint32_t data_len) {
    sensor_t *s = esp_camera_sensor_get();
    switch (cmd) {
        case 0x40: {
            ESP_LOGI(TAG, "server request img");
            camera_fb_t *fb = esp_camera_fb_get();
            ESP_LOGI(TAG, "from ov2640 get img, len:%d", fb->len);
            rs485_send_img(fb);
            esp_camera_fb_return(fb);
            break;
        }
        // set image size
        case 0x41: {
            ESP_LOGI(TAG, "server resize img");
            s->set_framesize(s, (framesize_t)data[0]);
            break;
        }
        // set img quality
        case 0x42: {
            ESP_LOGI(TAG, "server reset quality");
            s->set_quality(s, data[0]);
            break;
        }
        // set brightness
        case 0x43: {
            ESP_LOGI(TAG, "server reset brightness");
            s->set_brightness(s, data[0]);
            break;
        }
        // set saturation
        case 0x44: {
            ESP_LOGI(TAG, "server set saturation");
            s->set_saturation(s, data[0]);
            break;
        }
        // set gainceiling
        case 0x45: {
            ESP_LOGI(TAG, "server set gainceiling");
            s->set_gainceiling(s, (gainceiling_t)data[0]);
            break;
        }

        default:
            ESP_LOGW(TAG, "unkowm rs485 command");
            break;
    }
}

void app_main()
{
    esp_log_level_set("wifi", ESP_LOG_INFO);
    
    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ESP_ERROR_CHECK( nvs_flash_init() );
    }
    
    uart_init(115200);
 
    err = esp_camera_init(&camera_config);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed");
        for(;;) {
          vTaskDelay(100 / portTICK_RATE_MS);
        }
    } else {
      led_brightness(50);
    }

    sensor_t *s = esp_camera_sensor_get();
    s->set_framesize(s, FRAMESIZE_VGA);
    s->set_quality(s, 10);

    xTaskCreatePinnedToCore(uart_frame_task, "frame_get", configMINIMAL_STACK_SIZE + 2*1024, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(uart_rx_task, "rx_get", configMINIMAL_STACK_SIZE + 2*1024, NULL, 4, NULL, 0);
}