#ifndef _UART_HOST_H_
#define _UART_HOST_H_

#define UART_HOST       &UARTD3
#define UART_OSDK_BR    230400

#define UART_HOST_GIMBALINFO_FREQ   500U

void uartHost_init(void);

#endif
