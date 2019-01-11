/*
    buf ->  0: addr
            1: command
            crc16
*/
#ifndef RS485_H__
#define RS485_H__
#include "esp_camera.h"

extern void rs485_data_send(uint8_t *data, uint32_t data_len);
extern void rs485_send_img(camera_fb_t *fb);
extern void rs485_deal(uint8_t* buf, uint32_t buf_len);
// extern void  __attribute__((weak)) rs485_cmd_callback(uint8_t cmd, int8_t *data, uint32_t data_len);

#endif