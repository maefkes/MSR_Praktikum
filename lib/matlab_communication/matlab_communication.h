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

 typedef struct
 {
    // variables for sending
    uint16_t* parameter1;
    uint16_t* parameter2;
    uint16_t* parameter3;
    uint8_t command;

    //variables for parsing
    unsigned char motor1;
    unsigned char motor2;
    unsigned char motor3;
    unsigned char motor4;
 } matlab_communication_data_t;

typedef void (*matlabData_cb_t)(matlab_communication_data_t*);

/*** macros *************************************************************/
 /*** functions ************************************************************/
matlab_communication_error_t matlabCommunication_sendParameter(matlab_communication_t* matlabCom, matlab_communication_data_t* data);
matlab_communication_error_t matlabCommunication_getParserError(matlab_communication_t* matlabCom);
void matlabCommunication_registerDataCallback(matlab_communication_t* matlabCom, matlabData_cb_t cb);

matlab_communication_t* matlabCommunication_new(uart_t* uart);
void matlabCommunication_init(void);






#endif //MATLAB_COMMUNICATION_H