  #include <stdio.h>
  #include <stdbool.h>

  #include "stm32f2xx_hal.h"
  #include "matlab_communication.h"
  #include "uart.h"

// instance pointer
uart_t* uart4 = NULL;
matlab_communication_t* matlabCommunication = NULL;
static matlab_communication_practical_cmd_t _cmd = E_MATLABCOM_CMD_INITIAL;

volatile unsigned char a = 0;
volatile unsigned char b = 0;
volatile unsigned char c = 0;
volatile unsigned char d = 0;

volatile double ap = 0;
volatile double ai = 0;
volatile double ad = 0;

volatile double yp = 0;
volatile double yi = 0;
volatile double yd = 0;

volatile double sr = 0;
volatile double sp = 0;
volatile double sy = 0;


void matlabDataCallback(matlab_communication_data_t* data)
{
	if(data == NULL) {return;}

	switch(data->cmd)
	{
		case E_MATLABCOM_CMD_INITIAL:
			break;

		case E_MATLABCOM_CMD_SET_MOTOR_VALUE:  
		
		_cmd = data->cmd;
		a = data->motorData.motor1;
		b = data->motorData.motor2;
		c = data->motorData.motor3;
		d = data->motorData.motor4;
		break;
		
		case E_MATLABCOM_CMD_SET_PID_ANGLE_VALUES:
      _cmd = data->cmd;
      
			switch(data->currentPidAngleCmd)
			{
			case C_MATLABCOM_ROLL_PITCH_DATA:
			
				ap = data->pidAngleData.pPitch_Roll / 10;
				ai = data->pidAngleData.iPitch_Roll / 10;
				ad = data->pidAngleData.dPitch_Roll / 10;
				break;
			
			case C_MATLABCOM_YAW_DATA:
			
				yp = data->pidAngleData.pYaw / 10;
				yi = data->pidAngleData.iYaw / 10;
				yd = data->pidAngleData.dYaw / 10;
				break;
			
			case C_MATLABCOM_ANGLE_DATA:
			
			sr = data->pidAngleData.targetAngleRoll;
			sp = data->pidAngleData.targetAnglePitch;
			sy = data->pidAngleData.targetAngleYaw;
				break;
			default:
				//error
				break;
			}
			break;
	}
}

void QCSF_Control()
{
	
	
		// Gui practical with plot is active. USES CONTROLLER IN C
		if(_cmd == E_MATLABCOM_CMD_SET_PID_ANGLE_VALUES)
		{
     		 printf("gui practical reached");
		}
		// Matlab controller practical is active. WON'T USE CONTROLLER IN C
		else if(_cmd == E_MATLABCOM_CMD_SET_MOTOR_VALUE)
		{
			static bool _initialised = false;
			if (!_initialised)
			{
				a = 10;
				b = 10;
				c = 10;
				d = 10;
				_initialised = true;
			}
			//Motors_run(&motor);	
		}
		else
		{
			/* ADD NEW PRACTICAL ACTIONS HERE */
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
		matlabCommunication_registerDataCallback(matlabCommunication, matlabDataCallback);
	}

	volatile matlab_communication_error_t error;

	error = matlabCommunication_getParserError(matlabCommunication);
	printf("Error: %d\n", error);

	QCSF_Control();

	/*** main loop ***************************************************************/
	while(1)
  	{
		double x = 0;
		double y = 0;
		double z = 0;

		x = 10;
		y = 23;
		z = 105;

		matlabCommunication_sendImuData(matlabCommunication, x, y, z);
		HAL_Delay(3000);
	}
  	return 0; 
}
