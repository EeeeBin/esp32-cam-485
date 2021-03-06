#include "freertos/FreeRTOS.h"

const uint16_t crctalbeabs[] = { 
	0x0000, 0xCC01, 0xD801, 0x1400, 0xF001, 0x3C00, 0x2800, 0xE401, 
	0xA001, 0x6C00, 0x7800, 0xB401, 0x5000, 0x9C01, 0x8801, 0x4400 
};
 
uint16_t crc16tablefast(uint8_t *data, uint32_t len) 
{
	uint16_t crc = 0xffff; 
	uint8_t *ptr = data;
	uint32_t i;
	uint8_t ch;
 
	for (i = 0; i < len; i++) {
		ch = *ptr++;
		crc = crctalbeabs[(ch ^ crc) & 15] ^ (crc >> 4);
		crc = crctalbeabs[((ch >> 4) ^ crc) & 15] ^ (crc >> 4);
	} 
	
	return crc;
}
