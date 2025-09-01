/*************************************************************************
 * uart.h
 * Headerfile for uart.c
 * Created on: 20-Aug-2025 21:00:00
 * M. Schermutzki
 * This module...
 *************************************************************************/
#ifndef UART_H
#define UART_H
/*** includes ************************************************************/
#include <stdint.h>
#include <stddef.h>  // für size_t
#include <stdbool.h>

/*** local constants ******************************************************/
/*** macros *************************************************************/
/*** definitions ********************************************************/
typedef struct uart_s uart_t;
typedef void (*handler_cb_t)(uint8_t);

typedef enum
{
    UART_1,
    UART_2,
    UART_3,
    UART_4,
    UART_5,
    UART_6
} uart_port_t;
 
/*** local variables ******************************************************/
/*** funcions *************************************************************/
bool uart_sendBuffer(uart_t* uart, uint8_t *buffer, size_t len);
bool uart_sendByte(uart_t* uart, uint8_t data);
void uart_registerRxCallback(uart_t* uart, handler_cb_t cb);

uart_t* uart_init(uart_port_t port, uint32_t baudRate);

#endif // UART_H
