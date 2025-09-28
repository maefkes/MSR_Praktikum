/***************************************************************************
 * crc16.c
 * Created on: 24-Aug-2025 19:59:00
 * M. Schermutzki
 ***************************************************************************/
/*** includes **************************************************************/
#include "crc16.h"
#include <stdio.h>

/*** macros ***************************************************************/
#define C_CRC16_MAX_INSTANCES (5u)
/*** local constants ******************************************************/
/*** definitions **********************************************************/
struct crc16_s
{
    uint8_t startSign;
    uint8_t seperator;
    uint8_t endSign;
    uint16_t currentSum;
    bool isValidEntry;
};

/*** local variables ******************************************************/
static crc16_t _crc16Instances[C_CRC16_MAX_INSTANCES];
static bool _initialised = false;
/*** prototypes ***********************************************************/

/*** functions ************************************************************/
/***************************************************************************
 * This function 
 **************************************************************************/ 
void crc16_reset(crc16_t* crc16)
{
    if(!crc16 || crc16->isValidEntry == false) return;
    crc16->currentSum = 0xFFFF;  
}

/***************************************************************************
 * This function
 **************************************************************************/ 
void crc16_calculate(crc16_t* crc16, uint8_t data)
{
    if(!crc16 || crc16->isValidEntry == false) return;
    uint16_t crc = crc16->currentSum;
    crc ^= (uint16_t)data << 8;
    for(uint8_t i = 0; i < 8; i++)
    {
        if(crc & 0x8000)
            crc = (crc << 1) ^ 0x1021;  // CRC16-CCITT
        else
            crc <<= 1;
        crc &= 0xFFFF; // auf 16 Bit beschränken
    }
    crc16->currentSum = crc;
}

/***************************************************************************
 * This function
 **************************************************************************/ 
uint16_t crc16_get(crc16_t* crc16)
{
    if(!crc16 || crc16->isValidEntry == false) return 0;
    return crc16->currentSum;
}
/***************************************************************************
 * This function
 **************************************************************************/ 
void crc16_insertIntoDatagram(crc16_t* crc16, size_t outputSize, const char* input, char* output)
{
    if (!crc16 || !input || !output || outputSize == 0 || !crc16->isValidEntry) 
    {
        return; 
    }
    
        const char* datagramPtr;
        uint16_t checksumCalc;

        crc16_reset(crc16);

        for(datagramPtr = input; *datagramPtr != 0; ++datagramPtr)
        {
            crc16_calculate(crc16, *datagramPtr);
        }

        checksumCalc = crc16_get(crc16);

        snprintf(output, outputSize,
                "%c%s%c%hu%c",
                crc16->startSign,
                input,
                crc16->seperator,
                checksumCalc,
                crc16->endSign);
}
/***************************************************************************
 * This function
 **************************************************************************/ 
crc16_t* crc16_new(const uint8_t startSign, const uint8_t seperator, const uint8_t endSign)
{
    if (!_initialised) {return NULL;}

    for (uint8_t i = 0; i < C_CRC16_MAX_INSTANCES; i++)
    {
        if(!_crc16Instances[i].isValidEntry)
        {
            crc16_t* crc16 = &_crc16Instances[i];
            crc16->startSign   = startSign;
            crc16->seperator   = seperator;
            crc16->endSign     = endSign;
            crc16->currentSum  = 0xFFFF;
            crc16->isValidEntry = true;
            return crc16;
        }
    }
    return NULL; // no more slot available
}

/***************************************************************************
 * This function
 **************************************************************************/ 
void crc16_init()
{
    if (!_initialised)
    {
        for (uint8_t i = 0; i < C_CRC16_MAX_INSTANCES; i++)
        {
            _crc16Instances[i] = (crc16_t) {0};
        }
        _initialised = true;
    }

}
