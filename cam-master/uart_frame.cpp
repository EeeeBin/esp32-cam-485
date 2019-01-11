#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "string.h"

#include "uart_frame.h"

#define UART_NUM UART_NUM_1
#define RX_IDLE_TIME 2
 
volatile frame_state_n frame_state;
QueueHandle_t uart_buffer_quenue;
#define RX_BUF_SIZE 1024*80

void uart_init() {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE, 0, 0, NULL, 0);
}

void uart_frame_task(void *arg) {
    uint32_t buf_len = 0;
    uint32_t buf_len_last = 0;
    rx_buffer_t *rx_buffer;
    uint8_t num = 0;

    uart_buffer_quenue = xQueueCreate(5, sizeof(rx_buffer_t *));

    for(;;) {
        uart_get_buffered_data_len(UART_NUM, &buf_len);
        switch(frame_state) {
            case IDLE: 
                if(buf_len && buf_len != buf_len_last) {
                    buf_len_last = buf_len;
                    frame_state = WAIT_FINISH;
                } 
                break;
            
            case WAIT_FINISH:
                if(buf_len > buf_len_last) {
                    buf_len_last = buf_len;
                    num = 0;
                    break;
                } else {
                    num++;
                    if(num >= RX_IDLE_TIME) {
                        frame_state = FINISH;
                    } else {
                        break ;
                    }
                }

            case FINISH:
                num = 0;
                rx_buffer = (rx_buffer_t *)malloc(sizeof(rx_buffer_t));
                rx_buffer->buf = (uint8_t *)malloc(sizeof(uint8_t) * buf_len);
                rx_buffer->buf_size = buf_len;
                uart_read_bytes(UART_NUM, rx_buffer->buf, rx_buffer->buf_size, 1);
                xQueueSendToBack(uart_buffer_quenue, (void *)&rx_buffer, (TickType_t) 0);
                buf_len = 0;
                buf_len_last = 0;
                frame_state = IDLE;
                break;
        }

        vTaskDelay(10 / portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}