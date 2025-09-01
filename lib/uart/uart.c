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
static uint8_t _instanceCount = 0;

/*** definitions **********************************************************/
struct uart_s
{
    uint32_t baudRate;
    bool isValidEntry;
    uint8_t instanceNumber;
    uint8_t rxByte;

    UART_HandleTypeDef _huart;
    uart_port_t port;
    handler_cb_t rxCallback;
};

/*** local variables *****************************************************/
static uart_t _uartInstances[C_UART_MAX_INSTANCES];

/*** prototypes **********************************************************/
static bool _init_uartPort(uart_t* uart, uart_port_t port, uint32_t baudRate);
static bool _init_uartPort(uart_t* uart, uart_port_t port, uint32_t baudRate);
void _HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/*** functions ***********************************************************/
/*************************************************************************
 * This function initialises a uart port low level
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
            return false; 
    }

    uart->_huart.Init.BaudRate     = baudRate;
    uart->_huart.Init.WordLength   = UART_WORDLENGTH_8B;
    uart->_huart.Init.StopBits     = UART_STOPBITS_1;
    uart->_huart.Init.Parity       = UART_PARITY_NONE;
    uart->_huart.Init.Mode         = UART_MODE_TX_RX;
    uart->_huart.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    uart->_huart.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&uart->_huart) != HAL_OK)
    {
        return false;
    }

    // activate NVIC
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

    // start RX interrupt
    HAL_UART_Receive_IT(&uart->_huart, &uart->rxByte, 1);

    return true;
}

/*************************************************************************
* This function
************************************************************************/ 
void _HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    for (int i = 0; i < _instanceCount; i++)
    {
        if (&_uartInstances[i]._huart == huart)
        {
            uart_t* uart = &_uartInstances[i];
            if (uart->rxCallback)
            {
                uart->rxCallback(uart->rxByte);
            }
            // neuen RX-Interrupt starten
            HAL_UART_Receive_IT(&uart->_huart, &uart->rxByte, 1);
            break;
        }
    }
}

/*************************************************************************
* This function send single bytes 
************************************************************************/ 
bool uart_sendByte(uart_t* uart, uint8_t data)
{
    HAL_StatusTypeDef error = HAL_UART_Transmit(&uart->_huart, &data, 1, HAL_MAX_DELAY);
    if (error == HAL_OK)
    {
        return true;
    }
    return false;
}

bool uart_sendBuffer(uart_t* uart, uint8_t *buffer, size_t len)
{
    HAL_StatusTypeDef error = HAL_UART_Transmit(&uart->_huart, buffer, len, HAL_MAX_DELAY);
    if (error == HAL_OK)
    {
        return true;
    }
    return false;
}

/*************************************************************************
* This function registrates the callback function from higher layer
************************************************************************/ 
void uart_registerRxCallback(uart_t* uart, handler_cb_t cb)
{
    uart->rxCallback = cb;
}

/*************************************************************************
* This function
************************************************************************/ 
void USART1_IRQHandler(void)
{
    for (int i = 0; i < _instanceCount; i++)
    {
        if (_uartInstances[i].port == UART_1)
        {
            HAL_UART_IRQHandler(&_uartInstances[i]._huart);
            break;
        }
    }
}

void UART4_IRQHandler(void)
{
    for (int i = 0; i < _instanceCount; i++)
    {
        if (_uartInstances[i].port == UART_4)
        {
            HAL_UART_IRQHandler(&_uartInstances[i]._huart);
            break;
        }
    }
}
/*************************************************************************
* This function initalises an uart port
************************************************************************/ 
uart_t* uart_init(uart_port_t port, uint32_t baudRate)
{
    if (_instanceCount >= C_UART_MAX_INSTANCES) 
    {
        return NULL; 
    }

    uart_t* uart = &_uartInstances[_instanceCount];
    uart->instanceNumber = _instanceCount;  
    uart->baudRate = baudRate;
    uart->port = port;
    uart->rxByte = 0;
    uart->isValidEntry = _init_uartPort(uart, uart->port, baudRate);

    if (!uart->isValidEntry)
    {
        return NULL;
    }

    _instanceCount++;
    return uart;
}