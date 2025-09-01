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

typedef void (*matlabData_cb_t)(matlab_communication_t*);

typedef enum
{
    E_MATLABCOMERROR_OK,
    E_MATLABCOMERROR_NOK,
    E_MATLABCOMERROR_IN_PROGRESS,
    E_MATLABCOMERROR_INVALID_SIGN,
    E_MATLABCOMERROR_INVALID_POINTER,
    E_MATLABCOMERROR_INVALID_INSTANCE
} matlab_communication_error_t;

 /*
 typedef enum
 {

 } matlab_communication_current_protocol;
 */
/*** macros *************************************************************/
 /*** functions ************************************************************/
matlab_communication_error_t matlabCommunication_sendParameter(matlab_communication_t* matlabCom, uint16_t* value1, uint16_t* value2, uint16_t* value3);
matlab_communication_error_t matlabCommunication_getParserError(matlab_communication_t* matlabCom);
matlab_communication_t* matlabCommunication_init(uart_t* uart);






#endif //MATLAB_COMMUNICATION_H