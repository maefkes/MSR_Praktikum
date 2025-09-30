  #include <stdio.h>

  #include "stm32f2xx_hal.h"
  #include "matlab_communication.h"
  #include "uart.h"

// instance pointer
uart_t* uart4 = NULL;
matlab_communication_t* matlabCommunication = NULL;

void dataCallback(matlab_communication_data_t* data)
{
	if(data == NULL) {return;}

  switch(data->cmd)
  {
    case E_MATLABCOM_CMD_SET_MOTOR_VALUE:  
    
      data->motorData.motor1;
      data->motorData.motor2;
      data->motorData.motor3;
      data->motorData.motor4;
      break;
    
    case E_MATLABCOM_CMD_SET_PID_ANGLE_VALUES:
        switch(data->currentPidAngleCmd)
        {
          case C_MATLABCOM_ROLL_PITCH_DATA:
          
            data->pidAngleData.pPitch_Roll;
            data->pidAngleData.iPitch_Roll;
            data->pidAngleData.dPitch_Roll;
            break;
          
          case C_MATLABCOM_YAW_DATA:
          
            data->pidAngleData.pYaw;
            data->pidAngleData.iYaw;
            data->pidAngleData.dYaw;
            break;
          
          case C_MATLABCOM_ANGLE_DATA:
          
          data->pidAngleData.targetAngleRoll;
          data->pidAngleData.targetAnglePitch;
          data->pidAngleData.targetAngleYaw;
            break;
          default:
            //error
            break;
        }
        break;
	}
}

int main(void)
{
  /*** setup *******************************************************************/

  

  HAL_Init();

  matlabCommunication_init();

  // initialise instances
  if(uart4 == NULL && matlabCommunication == NULL)
  {
    uart4 = uart_new(UART_4, 57600);
    matlabCommunication = matlabCommunication_new(uart4);
  }

  matlabCommunication_registerDataCallback(matlabCommunication, dataCallback);
  volatile matlab_communication_error_t error;

  error = matlabCommunication_getParserError(matlabCommunication);
  printf("Error: %d\n", error);

  /*** main loop ***************************************************************/
  while(1)
  {
      HAL_Delay(3000);

  }
    
  return 0; 
}
