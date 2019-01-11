#include "M5Stack.h"
#include "userconfig.h"
#include "driver/uart.h"
#include "uart_frame.h"

extern uint16_t crc16tablefast(uint8_t *data, uint32_t len); 
static const char *TAG = "RS485";

SemaphoreHandle_t rs485_lock = NULL;

void rs485_mutex_init() {
  rs485_lock = xSemaphoreCreateMutex();
}

void __attribute__((weak)) img_deal(uint16_t addr, uint8_t* jpeg, uint32_t jpeg_len);

void rs485_deal(uint8_t* buf, uint32_t buf_len) {
  uint32_t crc16_data = 0;
  uint32_t buf_crc_data = 0;
  if(buf_len < 6) {
    ESP_LOGE(TAG, "get data error, data len is %d\r\n", buf_len);
    return ;
  }

  crc16_data = crc16tablefast(buf, buf_len - 2);
  buf_crc_data = buf[buf_len - 2] * 256 + buf[buf_len - 1];

  if(crc16_data != buf_crc_data) {
    ESP_LOGE(TAG, "crc error crc16 is 0x%x, but now 0x%x\r\n", crc16_data, buf_crc_data);
    return ;
  }

  if(buf[1] == 0x42) {
    img_deal(buf[0], &buf[2], buf_len - 4);
    ESP_LOGI(TAG, "get img\r\n");
  } else {
    ESP_LOGI(TAG, "test, command is 0x%x\r\n", buf[1]);
  }
}

void rs485_data_send(uint8_t *data, uint32_t data_len) {
  uint16_t crc16_data = 0;
  uint8_t crc16_tx[2] = { 0 };
  crc16_data = crc16tablefast(data, data_len);
  ESP_LOGI(TAG, "send msg, crc is 0x%x\r\n", crc16_data);
  crc16_tx[0] = crc16_data >> 8;
  crc16_tx[1] = (uint8_t)crc16_data;
  if(rs485_lock != NULL && frame_state == IDLE) {
    if(xSemaphoreTake(rs485_lock, 200 / portTICK_RATE_MS) == pdTRUE) {
      uart_write_bytes(UART_NUM_1, (char *)data, data_len);
      uart_write_bytes(UART_NUM_1, (char *)crc16_tx, 2);
      // frame end 20ms
      vTaskDelay(20 / portTICK_RATE_MS);
      xSemaphoreGive(rs485_lock);
    }
  } else {
    ESP_LOGE(TAG, "msg send fail, rs485 bus not idle");
  }

}

void cam_getimg(uint8_t addr) {
  uint8_t tx_buffer[4];
  tx_buffer[0] = addr;
  tx_buffer[1] = 0x40;
  tx_buffer[2] = 0x00;
  tx_buffer[3] = 0x00;
  rs485_data_send(tx_buffer, 4);
}

void cam_setsize(uint8_t addr, uint8_t data) {
  uint8_t tx_buffer[4];
  tx_buffer[0] = addr;
  tx_buffer[1] = 0x41;
  tx_buffer[2] = data;
  tx_buffer[3] = 0x01;
  rs485_data_send(tx_buffer, 4);
}

static void cam_setquality(uint8_t addr, uint8_t data) {
  uint8_t tx_buffer[4];
  tx_buffer[0] = addr;
  tx_buffer[1] = 0x42;
  tx_buffer[2] = data;
  tx_buffer[3] = 0x01;
  rs485_data_send(tx_buffer, 4);
}

static void cam_setbrightness(uint8_t addr, int8_t data) {
  uint8_t tx_buffer[4];
  tx_buffer[0] = addr;
  tx_buffer[1] = 0x43;
  tx_buffer[2] = (uint8_t)data;
  tx_buffer[3] = 0x01;
  rs485_data_send(tx_buffer, 4);
}

static void cam_saturation(uint8_t addr, int8_t data) {
  uint8_t tx_buffer[4];
  tx_buffer[0] = addr;
  tx_buffer[1] = 0x44;
  tx_buffer[2] = (uint8_t)data;
  tx_buffer[3] = 0x01;
  rs485_data_send(tx_buffer, 4);
}

static void cam_gainceiling(uint8_t addr, int8_t data) {
  uint8_t tx_buffer[4];
  tx_buffer[0] = addr;
  tx_buffer[1] = 0x45;
  tx_buffer[2] = (uint8_t)data;
  tx_buffer[3] = 0x01;
  rs485_data_send(tx_buffer, 4);
}
