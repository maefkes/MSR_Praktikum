/***************************************************************************
 * matlab_communication.h
 * Headerfile for matlab_communication.c
 * Created on: 24-Aug-2025 11:15:00
 * M. Schermutzki
 ***************************************************************************/
#ifndef MATLAB_COMMUNICATION_H
#define MATLAB_COMMUNICATION_H

/*** includes ************************************************************/
#include <stdint.h>
#include <stddef.h>
#include "uart.h"
/*** definitions ********************************************************/
#define C_MATLABCOM_ROLL_PITCH_DATA  (0x04)
#define C_MATLABCOM_YAW_DATA   (0x06)
#define C_MATLABCOM_ANGLE_DATA (0x07)

typedef struct matlab_communication_s matlab_communication_t;

typedef enum
{
    E_MATLABCOMERROR_OK,
    E_MATLABCOMERROR_NOK,
    E_MATLABCOMERROR_IN_PROGRESS,
    E_MATLABCOMERROR_INVALID_SIGN,
    E_MATLABCOMERROR_INVALID_POINTER,
    E_MATLABCOMERROR_INVALID_INSTANCE,
    E_MATLABCOMERROR_SEND,
    E_MATLABCOMERROR_UNK_CMD,
    E_MATLABCOMERROR_CHECKSUM_ERROR
} matlab_communication_error_t;

typedef enum
{
    E_MATLABCOM_CMD_INITIAL = 0,    // save initialisation in QCSF_API.c
    E_MATLABCOM_CMD_SET_MOTOR_VALUE   = 0x01,
    E_MATLABCOM_CMD_SET_PID_ANGLE_VALUES = 0x02
} matlab_communication_practical_cmd_t;

typedef struct
{
    uint16_t* parameter1;
    uint16_t* parameter2;
    uint16_t* parameter3;
    uint8_t command;
} matlab_communication_send_data_t;

typedef struct
{
    unsigned char motor1;
    unsigned char motor2;
    unsigned char motor3;
    unsigned char motor4;
}matlab_communication_motor_data_t;

typedef struct 
{
    volatile double pPitch_Roll;
    volatile double iPitch_Roll;
    volatile double dPitch_Roll;

    volatile double pYaw;
    volatile double iYaw;
    volatile double dYaw;

    volatile double targetAngleRoll;
    volatile double targetAnglePitch;
    volatile double targetAngleYaw;

} matlab_communication_Pid_Angle_data_t;

 typedef struct
 {
    matlab_communication_practical_cmd_t cmd;
    uint8_t currentPidAngleCmd;
    union
    {
        matlab_communication_send_data_t sendData;
        matlab_communication_motor_data_t motorData;
        matlab_communication_Pid_Angle_data_t pidAngleData;
    };
   
 } matlab_communication_data_t;

typedef void (*matlabData_cb_t)(matlab_communication_data_t*);

/*** macros *************************************************************/
 /*** functions ************************************************************/
matlab_communication_error_t matlabCommunication_sendParameter(matlab_communication_t* matlabCom, matlab_communication_data_t* data);
matlab_communication_error_t matlabCommunication_getParserError(matlab_communication_t* matlabCom);
void matlabCommunication_registerDataCallback(matlab_communication_t* matlabCom, matlabData_cb_t cb);
void matlabCommunication_sendImuData(matlab_communication_t* matlabCom, int16_t x, int16_t y, int16_t z);

matlab_communication_t* matlabCommunication_new(uart_t* uart);
void matlabCommunication_init(void);






#endif //MATLAB_COMMUNICATION_H