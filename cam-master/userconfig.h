#ifndef _USERCONFIG_H
#define _USERCONFIG_H

#include <Arduino.h>

typedef struct {
  uint8_t idle;
  uint32_t length;
  uint8_t *buf;
} jpeg_data_t;

#endif