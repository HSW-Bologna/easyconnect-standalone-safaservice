#ifndef RS485_H_INCLUDED
#define RS485_H_INCLUDED


#include <stdint.h>
#include <stdlib.h>


void rs485_init(void);
void rs485_write(const uint8_t *data, size_t len);
int  rs485_read(uint8_t *buffer, size_t len, unsigned long ms);
void rs485_flush(void);


#endif