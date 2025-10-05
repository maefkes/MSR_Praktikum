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

/*** definitions ********************************************************/
typedef struct uart_s uart_t;

// Callback-Typ mit optionalem Kontext
typedef void (*handler_cb_with_context_t)(void* context, uint8_t byte);

typedef enum
{
    UART_1,
    UART_2,
    UART_3,
    UART_4,
    UART_5,
    UART_6
} uart_port_t;

/*** functions ***********************************************************/
void uart_sendBuffer(uart_t* uart, const uint8_t *buffer, size_t len);
void uart_sendByte(uart_t* uart, uint8_t data);

// Callback mit Kontext registrieren
void uart_registerRxCallback(uart_t* uart, handler_cb_with_context_t cb, void* context);

uart_t* uart_new(uart_port_t port, uint32_t baudRate);
void uart_init(void);

#endif // UART_H
