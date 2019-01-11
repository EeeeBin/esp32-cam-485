#include <Arduino.h>
#include "M5Stack.h"
#include "uart_frame.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "userconfig.h"

#include "rs485.h"
#define CAM_ADDR 0x01

// extern uint16_t crc16tablefast(uint8_t *data, uint32_t len);

volatile jpeg_data_t jpeg_data_1;

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

void img_deal(uint16_t addr, uint8_t* jpeg, uint32_t jpeg_len) {
    M5.Lcd.drawJpg(jpeg, jpeg_len, 0, 0, 0, 0, 0, 0, JPEG_DIV_2);
}

void setup() {
  M5.begin(true, false, true);
  rs485_mutex_init();
  uart_init();
  xTaskCreatePinnedToCore(uart_frame_task, "frame_get", configMINIMAL_STACK_SIZE + 2*1024, NULL, 5, NULL, 0);
  xTaskCreatePinnedToCore(uart_rx_task, "rx_get", configMINIMAL_STACK_SIZE + 2*1024, NULL, 4, NULL, 0);
}

void loop() {
  M5.update();
  if(M5.BtnA.wasPressed()) {
      cam_getimg(0x01);
  }
  delay(20);
}
