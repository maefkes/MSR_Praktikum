/**************************************************************************
 * uart.c
 * Created on: 20-Aug-2025 21:00:00
 **************************************************************************/

/*** includes *************************************************************/
#include <stdbool.h>
#include <stddef.h>  
#include "uart.h"
#include "stm32f2xx_hal.h"

/*** macros ***************************************************************/
#define C_UART_MAX_INSTANCES  (6u)   

/*** local constants ******************************************************/
static bool _initialised = false;

/*** definitions **********************************************************/
struct uart_s
{
    uint32_t baudRate;
    bool isInUse;
    uint8_t instanceNumber;
    uint8_t rxByte;

    UART_HandleTypeDef _huart;
    uart_port_t port;
    handler_cb_with_context_t rxCallback;
    void* context;   // optionaler Kontext für den Callback
};

/*** local variables *****************************************************/
static uart_t _uartInstances[C_UART_MAX_INSTANCES];

/*** prototypes **********************************************************/
static bool _init_uartPort(uart_t* uart, uart_port_t port, uint32_t baudRate);

/*** functions ***********************************************************/

/*************************************************************************
 * Low-level Init eines Ports
 ************************************************************************/ 
static bool _init_uartPort(uart_t* uart, uart_port_t port, uint32_t baudRate)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    switch (port)
    {
        case UART_1:
            __HAL_RCC_USART1_CLK_ENABLE();
            __HAL_RCC_GPIOA_CLK_ENABLE();

            GPIO_InitStruct.Pin       = GPIO_PIN_9;  // TX
            GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
            GPIO_InitStruct.Pull      = GPIO_NOPULL;
            GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
            GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

            GPIO_InitStruct.Pin       = GPIO_PIN_10; // RX
            GPIO_InitStruct.Pull      = GPIO_PULLUP;
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

            uart->_huart.Instance = USART1;
            break;

        case UART_4:
            __HAL_RCC_UART4_CLK_ENABLE();
            __HAL_RCC_GPIOC_CLK_ENABLE();

            GPIO_InitStruct.Pin       = GPIO_PIN_10; // TX
            GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
            GPIO_InitStruct.Pull      = GPIO_NOPULL;
            GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
            GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
            HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

            GPIO_InitStruct.Pin       = GPIO_PIN_11; // RX
            GPIO_InitStruct.Pull      = GPIO_PULLUP;
            HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

            uart->_huart.Instance = UART4;
            break;

        default:
            return false; // ungültiger Port
    }

    uart->_huart.Init.BaudRate     = baudRate;
    uart->_huart.Init.WordLength   = UART_WORDLENGTH_8B;
    uart->_huart.Init.StopBits     = UART_STOPBITS_1;
    uart->_huart.Init.Parity       = UART_PARITY_NONE;
    uart->_huart.Init.Mode         = UART_MODE_TX_RX;
    uart->_huart.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    uart->_huart.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&uart->_huart) != HAL_OK)
        return false;

    // NVIC aktivieren
    switch (port)
    {
        case UART_1:
            HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
            HAL_NVIC_EnableIRQ(USART1_IRQn);
            break;
        case UART_4:
            HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
            HAL_NVIC_EnableIRQ(UART4_IRQn);
            break;
        default:
            return false;
    }

    // ersten RX-Interrupt starten
    HAL_UART_Receive_IT(&uart->_huart, &uart->rxByte, 1);

    return true;
}

/*************************************************************************
 * HAL Callback, wenn ein Byte empfangen wurde
 ************************************************************************/ 
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0; i < C_UART_MAX_INSTANCES; i++)
    {
        uart_t* uart = &_uartInstances[i];

        if (!uart->isInUse) continue;
        if (&uart->_huart != huart) continue;

        // User-Callback aufrufen, falls vorhanden
        if (uart->rxCallback)
            uart->rxCallback(uart->context, uart->rxByte);

        // neuen RX-Interrupt starten
        HAL_UART_Receive_IT(&uart->_huart, &uart->rxByte, 1);
        break;
    }
}

/*************************************************************************
 * Daten senden
 ************************************************************************/ 
void uart_sendByte(uart_t* uart, uint8_t data)
{
    HAL_UART_Transmit(&uart->_huart, &data, 1, HAL_MAX_DELAY);
}

void uart_sendBuffer(uart_t* uart, const uint8_t *buffer, size_t len)
{
    HAL_UART_Transmit(&uart->_huart, buffer, len, HAL_MAX_DELAY);
}

/*************************************************************************
 * Callback registrieren (inkl. Kontext)
 ************************************************************************/ 
void uart_registerRxCallback(uart_t* uart, handler_cb_with_context_t cb, void* context)
{
    uart->rxCallback = cb;
    uart->context = context;
}

/*************************************************************************
 * IRQ Handler für alle Ports dynamisch
 ************************************************************************/ 
void USART1_IRQHandler(void)
{
    for (uint8_t i = 0; i < C_UART_MAX_INSTANCES; i++)
    {
        uart_t* uart = &_uartInstances[i];

        if (!uart->isInUse) continue;
        if (uart->port != UART_1) continue;

        HAL_UART_IRQHandler(&uart->_huart);
        break;
    }
}

void UART4_IRQHandler(void)
{
    for (uint8_t i = 0; i < C_UART_MAX_INSTANCES; i++)
    {
        uart_t* uart = &_uartInstances[i];

        if (!uart->isInUse) continue;
        if (uart->port != UART_4) continue;

        HAL_UART_IRQHandler(&uart->_huart);
        break;
    }
}

/*************************************************************************
 * UART initialisieren
 ************************************************************************/ 
uart_t* uart_new(uart_port_t port, uint32_t baudRate)
{
    if (!_initialised) return NULL;

    for (uint8_t i = 0; i < C_UART_MAX_INSTANCES; i++) 
    {
        if (!_uartInstances[i].isInUse) 
        {
            uart_t* uart = &_uartInstances[i];

            uart->instanceNumber = i;
            uart->baudRate = baudRate;
            uart->port = port;
            uart->rxByte = 0;
            uart->rxCallback = NULL;
            uart->context = NULL;

            uart->isInUse = _init_uartPort(uart, port, baudRate);
            if (!uart->isInUse)
            {
                *uart = (uart_t){0};
                return NULL;
            }

            return uart;
        }
    }

    return NULL;
}

void uart_init(void)
{
    if (!_initialised)
    {
        for (uint8_t i = 0; i < C_UART_MAX_INSTANCES; i++)
        {
            _uartInstances[i].isInUse = false;
        }
        _initialised = true;
    }
}
