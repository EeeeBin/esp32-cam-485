#ifndef _RS485_H_
#define _RS485_H_

#include <Arduino.h>
extern void rs485_deal(uint8_t* buf, uint32_t buf_len);
extern void rs485_mutex_init();

extern void cam_getimg(uint8_t);
#endif