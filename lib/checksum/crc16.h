/*************************************************************************
 * additiv_checksum.h
 * Headerfile for additiv_checksum.c
 * Created on: 24-Aug-2025 19:59:00
 * M. Schermutzki
 * This module...
 *************************************************************************/
#ifndef CRC16_H
#define CRC16_H
/*** includes ************************************************************/
#include <stdint.h>
#include <stddef.h>  
#include <stdbool.h>
/*** local constants ******************************************************/
/*** macros *************************************************************/
/*** definitions ********************************************************/
typedef struct crc16_s crc16_t;
/*** local variables ******************************************************/
/*** funcions *************************************************************/
void crc16_reset(crc16_t* checksum);
void crc16_calculate(crc16_t* checksum, uint8_t data);
uint16_t crc16_get(crc16_t* checksum);
void crc16_insertIntoDatagram(crc16_t* crc16, size_t outputSize, const char* input, char* output);

crc16_t* crc16_new(const uint8_t startSign, const uint8_t seperator, const uint8_t endSign);
void crc16_init(void);

#endif // CRC16_H
