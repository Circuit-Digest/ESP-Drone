/**********************************************************************
 *
 * Filename:    crc.h
 * 
 * Description: A header file describing the various CRC standards.
 *
 * Notes:  See crc.c     
 *
 *
 * Copyright (c) 2012 Bitcraze AB
 * Copyright (c) 2000 by Michael Barr.  This software is placed into
 * the public domain and may be used for any purpose.  However, this
 * notice must not be changed or removed and no warranty is either
 * expressed or implied by its publication or distribution.
 **********************************************************************/
/**
 * @file 
 * Efficient implementation of various CRC standards.
 *
 * Two CRC calculation function are implemented: flow and fast. The slow version
 * is espcially inefficient but virtually use no RAM at all. The fast function
 * is more efficient cpu-wise but require from 512 to 1KiB of RAM located lookup
 * table (might be possible to move the table into FLASH, but then it would take
 * 1K of flash ...).
 * 
 * crcInit() shall be used *only* if the fast version of CRC is used.
 *
 * The CRC algorythm to use is choosen in crc.h.
 */

#ifndef _crc_h
#define _crc_h

#include <stdint.h>

#define FALSE	0
#define TRUE	!FALSE

/**
 * Select the CRC standard from the list that follows.
 */
#define CRC32


#if defined(CRC_CCITT)

typedef uint16_t crc;

#define CRC_NAME			"CRC-CCITT"
#define POLYNOMIAL			0x1021
#define INITIAL_REMAINDER	0xFFFF
#define FINAL_XOR_VALUE		0x0000
#define REFLECT_DATA		FALSE
#define REFLECT_REMAINDER	FALSE
#define CHECK_VALUE			0x29B1

#elif defined(CRC16)

typedef uint16_t crc;

#define CRC_NAME			"CRC-16"
#define POLYNOMIAL			0x8005
#define INITIAL_REMAINDER	0x0000
#define FINAL_XOR_VALUE		0x0000
#define REFLECT_DATA		TRUE
#define REFLECT_REMAINDER	TRUE
#define CHECK_VALUE			0xBB3D

#elif defined(CRC32)

typedef uint32_t crc;

#define CRC_NAME			"CRC-32"
#define POLYNOMIAL			0x04C11DB7
#define INITIAL_REMAINDER	0xFFFFFFFF
#define FINAL_XOR_VALUE		0xFFFFFFFF
#define REFLECT_DATA		TRUE
#define REFLECT_REMAINDER	TRUE
#define CHECK_VALUE			0xCBF43926

#else

#error "One of CRC_CCITT, CRC16, or CRC32 must be #define'd."

#endif

/**
 * Init function that must be called before using crcFast().
 * This function is useless for crcSlow() and shall not be called if only
 * crcSlow() is used!
 *
 * @note Using this function will use 1KiB of BSS ram!
 */
void  crcInit(void);

/**
 * Slow implementation of CRC.
 * 
 * This function trades speeds for memory usage as it does not require
 * pre-calculated tables but executes some code for each data bits. Good for one
 * time CRC calculation. Bad for data communication check.
 *
 * @param datas Pointer to the data buffer onto the CRC will be calculated.
 * @param nBytes Number of bytes to calculate the CRC from.
 * @return The CRC value.
 */
crc   crcSlow(void * datas, int nBytes);

/**
 * Fast implementation of CRC.
 * 
 * This function trades memory usage for speed as it requires pre-calculated
 * table (calculated by crcInit()). The requirement is 512Bytes for 16bits CRC
 * and 1024Bytes for 32bits CRC.
 *
 * @param datas Pointer to the data buffer onto the CRC will be calculated.
 * @param nBytes Number of bytes to calculate the CRC from.
 * @return The CRC value.
 */
crc   crcFast(void * datas, int nBytes);


#endif /* _crc_h */
