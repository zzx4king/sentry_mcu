#ifndef __USART_DEVICE_H
#define __USART_DEVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

#define  DMA_BUF_SIZE    256 


struct UART_Device {
    char *name;
		int (*Init)( struct UART_Device *pDev, int baud, char parity, int data_bit, int stop_bit);
		int (*Send)( struct UART_Device *pDev, uint8_t *datas, uint32_t len, int timeout);
		int (*RecvByte)( struct UART_Device *pDev, uint8_t *data, int timeout);
};

struct UART_Device *GetUARTDevice(char *name);

#ifdef __cplusplus
}
#endif

#endif /* __USART_DEVICE_H */
