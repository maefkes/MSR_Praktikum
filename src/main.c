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

    uint16_t a = 10;
    uint16_t b = 20;
    uint16_t c = 30;

    matlabCommunication_sendParameter(matlabCommunication, &a, &b, &c);

    volatile matlab_communication_error_t error;

    error = matlabCommunication_parsingResponse(matlabCommunication);
    printf("Error: %d\n", error);

    while(1)
    {
        HAL_Delay(3000);
    }
    
    return 0; // optional, da embedded meist nie erreicht
}
