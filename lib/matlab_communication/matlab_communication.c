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
#define C_MATLABCOM_MAX_INSTANCES    (2u)
#define C_MATLABCOM_MAX_BUFFER_SIZE  (30u)

// parser commands
#define CMD_MOTOR_VALUES (0x01)
#define CMD_ 0x02

// protocol frame data
#define C_MATLABCOM_STX  (0x02) // start sign
#define C_MATLABCOM_US   (0x1F) // separator
#define C_MATLABCOM_ETX  (0x03) // end sign

/*** definitions **********************************************************/
typedef void (*parser_state_t)(matlab_communication_t* matlabCom, const uint8_t sign);

struct matlab_communication_s
{
    matlab_communication_error_t error;
    matlab_communication_data_t data;
    parser_state_t currentState;
    matlabData_cb_t dataCallback;
    uart_t* communication;
    crc16_t* checksum;
    uint8_t fieldIndex;
    int32_t numContainer;
    uint8_t currentCommand;
    bool isValidEntry;
};

/*** local variables ******************************************************/
static matlab_communication_t _matlabComInstances[C_MATLABCOM_MAX_INSTANCES];
static bool _initialised = false;
matlabData_cb_t _dataCallback = NULL;

/*** prototypes ***********************************************************/
static bool _asciiToNumber(matlab_communication_t* matlabCom, const char input);

// state machine functions
static void _parserState_idle(matlab_communication_t* matlabCom, uint8_t sign);
static void _parserState_readCommand(matlab_communication_t* matlabCom, uint8_t sign);
static void _parserState_readMotorValues(matlab_communication_t* matlabCom, uint8_t sign);
static void _parserState_validateChecksum(matlab_communication_t* matlabCom, uint8_t sign);


/*** functions ************************************************************/

/***************************************************************************
 * Transform ascii signs into numbers
 **************************************************************************/ 
static bool _asciiToNumber(matlab_communication_t* matlabCom, const char input)
{
    uint8_t digit;

    if (input >= '0' && input <= '9') {
        digit = input - '0';
    }
    else if (input >= 'A' && input <= 'F') {
        digit = input - 'A' + 0xA;
    }
    else if (input >= 'a' && input <= 'f') {
        digit = input - 'a' + 0xA;
    }
    else {
        return false;
    }

    matlabCom->numContainer = matlabCom->numContainer * 16 + digit;
    return true;
}

/***************************************************************************
 * State machine: idle
 **************************************************************************/ 
static void _parserState_idle(matlab_communication_t* matlabCom, uint8_t sign)
{
    if (sign == C_MATLABCOM_STX)
    {
        crc16_reset(matlabCom->checksum);
        matlabCom->numContainer = 0;
        matlabCom->fieldIndex = 0;
        matlabCom->currentState = _parserState_readCommand;
        matlabCom->error = E_MATLABCOMERROR_IN_PROGRESS;
    }
}

/***************************************************************************
 * State machine: read command
 **************************************************************************/ 
static void _parserState_readCommand(matlab_communication_t* matlabCom, uint8_t sign)
{
    if (matlabCom->error != E_MATLABCOMERROR_IN_PROGRESS) return;

    if (sign == C_MATLABCOM_US)
    {
        // US auch in CRC einbeziehen
        crc16_calculate(matlabCom->checksum, sign);

        // Command fertig
        if (matlabCom->numContainer == CMD_MOTOR_VALUES)
        {
            matlabCom->currentCommand = matlabCom->numContainer;
            matlabCom->numContainer = 0;
            matlabCom->currentState = _parserState_readMotorValues;
        }
        else
        {
            matlabCom->error = E_MATLABCOMERROR_UNK_CMD;
        }
    }
    else
    {
        bool success = _asciiToNumber(matlabCom, sign);
        if (success)
        {
            crc16_calculate(matlabCom->checksum, sign);
        }
        else
        {
            matlabCom->error = E_MATLABCOMERROR_INVALID_SIGN;
        }
    }
}

/***************************************************************************
 * State machine: read motor values
 **************************************************************************/ 
static void _parserState_readMotorValues(matlab_communication_t* matlabCom, uint8_t sign)
{
    if (matlabCom->error != E_MATLABCOMERROR_IN_PROGRESS) return;
    if (matlabCom->currentCommand != CMD_MOTOR_VALUES) return;

    if (sign == C_MATLABCOM_US)
    {
        // US auch in CRC einbeziehen
        crc16_calculate(matlabCom->checksum, sign);

        // Ein Feld abgeschlossen
        switch (matlabCom->fieldIndex)
        {
            case 0: matlabCom->data.motor1 = matlabCom->numContainer; break;
            case 1: matlabCom->data.motor2 = matlabCom->numContainer; break;
            case 2: matlabCom->data.motor3 = matlabCom->numContainer; break;
            case 3: matlabCom->data.motor4 = matlabCom->numContainer; break;
            default: matlabCom->error = E_MATLABCOMERROR_NOK; return;
        }

        matlabCom->numContainer = 0;
        matlabCom->fieldIndex++;

        if (matlabCom->fieldIndex > 3)
        {
            matlabCom->fieldIndex = 0;
            matlabCom->numContainer = 0;
            matlabCom->currentState = _parserState_validateChecksum;
        }
    }
    else
    {
        bool success = _asciiToNumber(matlabCom, sign);
        if (success)
        {
            crc16_calculate(matlabCom->checksum, sign);
        }
        else
        {
            matlabCom->error = E_MATLABCOMERROR_INVALID_SIGN;
        }
    }
}


/***************************************************************************
 * State machine: validate checksum
 **************************************************************************/ 
static void _parserState_validateChecksum(matlab_communication_t* matlabCom, uint8_t sign)
{
    if (matlabCom->error != E_MATLABCOMERROR_IN_PROGRESS) return;

    bool success = true;
    

    if (sign == C_MATLABCOM_ETX)
    {
        uint16_t calculatedCrc = crc16_get(matlabCom->checksum);
        uint16_t sendedCrc = (uint16_t)matlabCom->numContainer;
       
        if (calculatedCrc == sendedCrc)
        {
            if (matlabCom->dataCallback)
            {
                matlabCom->dataCallback(&matlabCom->data);
            }
            matlabCom->error = E_MATLABCOMERROR_OK;
        }
        else
        {
            matlabCom->error = E_MATLABCOMERROR_CHECKSUM_ERROR;
        }

        // Reset state machine
        matlabCom->numContainer = 0;
        calculatedCrc = 0;
        matlabCom->fieldIndex = 0;
        matlabCom->currentState = _parserState_idle;
    }
    else
    {
        success = _asciiToNumber(matlabCom, sign);
        if (!success)
        {
            matlabCom->error = E_MATLABCOMERROR_INVALID_SIGN;
        }
    }
}


/***************************************************************************
 * UART RX wrapper for single instance
 **************************************************************************/ 
static void _uartRxWrapper(void* context, uint8_t byte)
{
    matlab_communication_t* matlabCom = (matlab_communication_t*) context;

    if (matlabCom && matlabCom->isValidEntry && matlabCom->currentState)
    {
        matlabCom->currentState(matlabCom, byte);
    }
}

/***************************************************************************
 * Send MATLAB parameters
 **************************************************************************/ 
matlab_communication_error_t matlabCommunication_sendParameter(matlab_communication_t* matlabCom, matlab_communication_data_t* data)
{
    if(matlabCom == NULL || data == NULL) return E_MATLABCOMERROR_INVALID_POINTER;

    char datagram[C_MATLABCOM_MAX_BUFFER_SIZE] = {0};
    char readyToSend[C_MATLABCOM_MAX_BUFFER_SIZE] = {0};

    if(data->parameter1 && data->parameter2 && data->parameter3)
    {
        snprintf(datagram, sizeof(datagram), "%hu%c%hu%c%hu%c%hu",
                 data->command, C_MATLABCOM_US,
                 *data->parameter1, C_MATLABCOM_US,
                 *data->parameter2, C_MATLABCOM_US,
                 *data->parameter3);

        crc16_insertIntoDatagram(matlabCom->checksum, sizeof(readyToSend), datagram, readyToSend);
        uart_sendBuffer(matlabCom->communication, (uint8_t*)readyToSend, strlen(readyToSend));
    }
    else
    {
        matlabCom->error = E_MATLABCOMERROR_NOK;
        return matlabCom->error;
    }

    return E_MATLABCOMERROR_OK;
}

/***************************************************************************
 * Return parser error
 **************************************************************************/
matlab_communication_error_t matlabCommunication_getParserError(matlab_communication_t* matlabCom)
{
    if (!matlabCom) return E_MATLABCOMERROR_INVALID_POINTER;
    if (!matlabCom->isValidEntry) return E_MATLABCOMERROR_INVALID_INSTANCE;

    return matlabCom->error;
}
/***************************************************************************
 * 
 **************************************************************************/
void matlabCommunication_registerDataCallback(matlab_communication_t* matlabCom, matlabData_cb_t cb) 
{
    if (matlabCom) matlabCom->dataCallback = cb;
}

/***************************************************************************
 * Create new MATLAB communication instance
 **************************************************************************/ 
matlab_communication_t* matlabCommunication_new(uart_t* uart)
{
    if (!uart || !_initialised) return NULL;

    for (uint8_t i = 0; i < C_MATLABCOM_MAX_INSTANCES; i++)
    {
        if(!_matlabComInstances[i].isValidEntry)
        {
            matlab_communication_t* matlabCom = &_matlabComInstances[i];

            matlabCom->communication = uart;
            matlabCom->checksum = crc16_new(C_MATLABCOM_STX, C_MATLABCOM_US, C_MATLABCOM_ETX);
            matlabCom->fieldIndex = 0;
            matlabCom->numContainer = 0;
            matlabCom->currentState = _parserState_idle;
            matlabCom->error = E_MATLABCOMERROR_OK;
            matlabCom->isValidEntry = true;

            // Register UART RX callback with context
            uart_registerRxCallback(uart, _uartRxWrapper, matlabCom);
            return matlabCom;
        }
    }

    return NULL;
}

/***************************************************************************
 * Initialize MATLAB communication system
 **************************************************************************/ 
void matlabCommunication_init()
{
    if (!_initialised)
    {
        for (uint8_t i = 0; i < C_MATLABCOM_MAX_INSTANCES; i++)
        {
            _matlabComInstances[i] = (matlab_communication_t){0};
        }
        uart_init();
        crc16_init();
        _initialised = true;
    }
}
