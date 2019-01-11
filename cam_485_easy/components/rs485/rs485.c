#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_camera.h"
#include "esp_log.h"

extern uint16_t crc16tablefast(uint8_t *data, uint32_t len);
extern uint16_t crc16tablefast_muti(uint8_t *data, uint32_t len, uint8_t *data1,
                                    uint32_t len1);

#define ADDR 0x01
static const char *TAG = "RS485";

void rs485_data_send(uint8_t *data, uint32_t data_len) {
    uint16_t crc16_data = 0;
    uint8_t crc16_tx[2] = {0};
    crc16_data = crc16tablefast(data, data_len);
    crc16_tx[0] = crc16_data >> 8;
    crc16_tx[1] = (uint8_t)crc16_data;
    uart_write_bytes(UART_NUM_1, (char *)data, data_len);
    uart_write_bytes(UART_NUM_1, (char *)crc16_tx, 2);
}

void rs485_send_img(camera_fb_t *fb) {
    uint16_t crc16_data = 0;
    uint8_t crc16_tx[2] = {0};
    uint8_t tx_buffer[2];
    tx_buffer[0] = ADDR;
    tx_buffer[1] = 0x42;
    crc16_data = crc16tablefast_muti(tx_buffer, 2, fb->buf, fb->len);
    crc16_tx[0] = crc16_data >> 8;
    crc16_tx[1] = (uint8_t)crc16_data;
    uart_write_bytes(UART_NUM_1, (char *)tx_buffer, 2);
    uart_write_bytes(UART_NUM_1, (char *)fb->buf, fb->len);
    uart_write_bytes(UART_NUM_1, (char *)crc16_tx, 2);
}

void __attribute__((weak)) rs485_cmd_callback(uint8_t cmd, int8_t *data, uint32_t data_len);

void rs485_deal(uint8_t *buf, uint32_t buf_len) {
    uint32_t crc16_data = 0;
    uint32_t buf_crc_data = 0;
    if (buf_len < 6) {
        ESP_LOGE(TAG, "get data error, data len is %d", buf_len);
        return;
    }

    crc16_data = crc16tablefast(buf, buf_len - 2);
    buf_crc_data = buf[buf_len - 2] * 256 + buf[buf_len - 1];

    if (crc16_data != buf_crc_data) {
        ESP_LOGE(TAG, "crc error crc16 is 0x%x, but now 0x%x", crc16_data,
                 buf_crc_data);
        return;
    }

    if (buf[0] == ADDR || buf[0] == 0x00) {
        rs485_cmd_callback(buf[1], (int8_t *)&buf[2], buf_len - 4);
    }
}
