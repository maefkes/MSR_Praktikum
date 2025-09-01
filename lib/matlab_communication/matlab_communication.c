/***************************************************************************
 * matlab_communication.c
 * Created on: 24-Aug-2025 11:15:00
 * M. Schermutzki
 ***************************************************************************/
/*** includes **************************************************************/
#include "matlab_communication.h"
#include "crc16.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/*** macros ***************************************************************/
#define C_MATLABCOM_MAX_INSTANCES (2u)
#define C_MATLABCOM_MAX_BUFFER_SIZE (30u)
// protocol frame data
#define C_MATLABCOM_STX  (0x02) // start sign
#define C_MATLABCOM_US   (0x1F) // seperator
#define C_MATLABCOM_ETX  (0x03) // end sign

/*** definitions **********************************************************/
typedef void (*parser_state_t)(matlab_communication_t* this, const uint8_t sign);

struct matlab_communication_s
{
    matlab_communication_error_t error;
    parser_state_t currentState;
    uart_t* communication;
    crc16_t* checksum;
    uint8_t fieldIndex;
    uint32_t numContainer;
    uint8_t currentCommand;
    bool initialised;

};

/*** local constants ******************************************************/


/*** local variables ******************************************************/
static matlab_communication_t _matlabComInstances[C_MATLABCOM_MAX_INSTANCES];
static uint8_t _instanceCount = 0;
/*** prototypes ***********************************************************/
static bool _asciiToNumber(matlab_communication_t* matlabCom, const char input);

// state machine functions
static void _parserState_idle(matlab_communication_t* matlabCom, uint8_t sign);
static void _parserState_readCommand(matlab_communication_t* matlabCom, uint8_t sign);
static void _parserState_validateChecksum(matlab_communication_t* matlabCom, uint8_t sign);

/*** functions ************************************************************/
/***************************************************************************
 * This function transforms ascii signs into numbers
 **************************************************************************/ 
static bool _asciiToNumber(matlab_communication_t* matlabCom, const char input)
{
    if (input >= '0' && input <= '9')
    {
        uint8_t digit = input - '0';
        matlabCom->numContainer = matlabCom->numContainer * 16 + digit;
        return true;
    }
    
    else if (input >= 'A' && input <= 'F')
    {
        uint8_t digit = input - 'A' + 0xA;
        matlabCom->numContainer = matlabCom->numContainer * 16 + digit;
        return true;
    }
    return false;
}
/***************************************************************************
 * This function 
 **************************************************************************/ 
static void _parserState_idle(matlab_communication_t* matlabCom, uint8_t sign)
{
    if (sign == C_MATLABCOM_STX)
    {
        crc16_reset(matlabCom->checksum);
        matlabCom->currentState = _parserState_readCommand;
        matlabCom->error = E_MATLABCOMERROR_IN_PROGRESS;
    }
    else
    {
        matlabCom->error = E_MATLABCOMERROR_INVALID_SIGN;
    }
}
/***************************************************************************
 * This function
 **************************************************************************/ 
static void _parserState_readCommand(matlab_communication_t* matlabCom, uint8_t sign)
{
    if(matlabCom->error != E_MATLABCOMERROR_IN_PROGRESS) return;

    bool success = _asciiToNumber(matlabCom, sign);

    if (success)
    {
        // calculates crc only for valide signs
        crc16_calculate(matlabCom->checksum, sign);

    }
    else
    {
        matlabCom->error = E_MATLABCOMERROR_INVALID_SIGN;
    }
}
 /***************************************************************************
 * This function
 **************************************************************************/ 
/***************************************************************************
 * Generic callback for all MATLAB communication instances. 
 * For each received byte, it calls the current state function of 
 * all instances with a valid UART pointer, ensuring independent 
 * and instance-specific parsing. 
 **************************************************************************/ 
static void _uartRxWrapper(uint8_t byte)
{
    for (uint8_t i = 0; i < _instanceCount; i++)
    {
        // State-Funktion aufrufen
        _matlabComInstances[i].currentState(&_matlabComInstances[i], byte);
    }
}

/***************************************************************************
 * This function
 **************************************************************************/ 
matlab_communication_error_t matlabCommunication_sendParameter(matlab_communication_t* matlabCom, 
                                                               uint16_t* value1, 
                                                               uint16_t* value2, 
                                                               uint16_t* value3)
{
    char datagram[C_MATLABCOM_MAX_BUFFER_SIZE] = {0};
    char readyToSend[C_MATLABCOM_MAX_BUFFER_SIZE] = {0};

    if(value1 != NULL && value2 != NULL && value3 != NULL)
    {
        snprintf(datagram, sizeof(datagram), "%hu%c%hu%c%hu",
         *value1, C_MATLABCOM_US,
         *value2, C_MATLABCOM_US,
         *value3);

    crc16_insertIntoDatagram(matlabCom->checksum, sizeof(readyToSend), datagram, readyToSend);
    uart_sendBuffer(matlabCom->communication, (uint8_t*)readyToSend, strlen(readyToSend));
    }
    else if(value1 != NULL && value2 != NULL && value3 == NULL)
    {
        
    }
    else if(value1 != NULL && value2 == NULL && value3 == NULL)
    {
    }
    else
    {
        return E_MATLABCOMERROR_NOK;
    }
}
/***************************************************************************
 * This functions returns parsing errors
 **************************************************************************/
matlab_communication_error_t matlabCommunication_getParserError(matlab_communication_t* matlabCom)
{
    if (matlabCom == NULL) {return E_MATLABCOMERROR_INVALID_POINTER;}

    if(matlabCom->initialised)
    {
        return matlabCom->error;
    }
    else
    {
        return E_MATLABCOMERROR_INVALID_INSTANCE;
    }
}

/***************************************************************************
 * This function
 **************************************************************************/ 
matlab_communication_t* matlabCommunication_init(uart_t* uart)
{
    if (uart == NULL || _instanceCount >= C_MATLABCOM_MAX_INSTANCES) {return NULL;}

    // save the instances into an local array
    matlab_communication_t* matlabCom = &_matlabComInstances[_instanceCount];

    matlabCom->communication = uart;
    matlabCom->checksum = crc16_init(C_MATLABCOM_STX, C_MATLABCOM_US, C_MATLABCOM_ETX);
    matlabCom->fieldIndex = 0;
    matlabCom->numContainer = 0;
    matlabCom->currentState = _parserState_idle;
    matlabCom->error = E_MATLABCOMERROR_OK;
    matlabCom->initialised = true;

    // Register a global UART RX callback to automatically handle incoming bytes 
    // and feed them into the MATLAB parser state machine.
    uart_registerRxCallback(uart, _uartRxWrapper);

    _instanceCount++;
     return  matlabCom;
}

