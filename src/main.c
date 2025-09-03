  #include <stdio.h>

  #include "stm32f2xx_hal.h"
  #include "matlab_communication.h"
  #include "uart.h"

// instance pointer
uart_t* uart4;
matlab_communication_t* matlabCommunication;

int main(void)
{
    HAL_Init();

    // initialise instances
    uart4 = uart_init(UART_4, 57600);
    matlabCommunication = matlabCommunication_init(uart4);

    uint16_t p1 = 10;
    uint16_t p2 = 20;
    uint16_t p3 = 30;

    matlab_communication_data_t data;
    data.command    = CMD_P1;
    data.parameter1 = &p1;
    data.parameter2 = &p2;
    data.parameter3 = &p3;

    matlabCommunication_sendParameter(matlabCommunication, &data);

    volatile matlab_communication_error_t error;

    error = matlabCommunication_getParserError(matlabCommunication);
    printf("Error: %d\n", error);

    while(1)
    {
        HAL_Delay(3000);
    }
    
    return 0; 
}
