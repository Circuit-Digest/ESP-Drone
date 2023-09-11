// I2Cdev library collection - HMC5883L I2C device class header file
// Based on Honeywell HMC5883L datasheet, 10/2010 (Form #900405 Rev B)
// 6/12/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg
Adapted to Crazyflie FW by Bitcraze

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifndef HMC5883L_H_
#define HMC5883L_H_
#include <stdbool.h>
#include "i2cdev.h"

#define HMC5883L_ADDRESS            0x1E // this device only has one address
#define HMC5883L_DEFAULT_ADDRESS    0x1E

#define HMC5883L_RA_CONFIG_A        0x00
#define HMC5883L_RA_CONFIG_B        0x01
#define HMC5883L_RA_MODE            0x02
#define HMC5883L_RA_DATAX_H         0x03
#define HMC5883L_RA_DATAX_L         0x04
#define HMC5883L_RA_DATAZ_H         0x05
#define HMC5883L_RA_DATAZ_L         0x06
#define HMC5883L_RA_DATAY_H         0x07
#define HMC5883L_RA_DATAY_L         0x08
#define HMC5883L_RA_STATUS          0x09
#define HMC5883L_RA_ID_A            0x0A
#define HMC5883L_RA_ID_B            0x0B
#define HMC5883L_RA_ID_C            0x0C

#define HMC5883L_CRA_AVERAGE_BIT    6
#define HMC5883L_CRA_AVERAGE_LENGTH 2
#define HMC5883L_CRA_RATE_BIT       4
#define HMC5883L_CRA_RATE_LENGTH    3
#define HMC5883L_CRA_BIAS_BIT       1
#define HMC5883L_CRA_BIAS_LENGTH    2

#define HMC5883L_AVERAGING_1        0x00
#define HMC5883L_AVERAGING_2        0x01
#define HMC5883L_AVERAGING_4        0x02
#define HMC5883L_AVERAGING_8        0x03

#define HMC5883L_RATE_0P75          0x00
#define HMC5883L_RATE_1P5           0x01
#define HMC5883L_RATE_3             0x02
#define HMC5883L_RATE_7P5           0x03
#define HMC5883L_RATE_15            0x04
#define HMC5883L_RATE_30            0x05
#define HMC5883L_RATE_75            0x06

#define HMC5883L_BIAS_NORMAL        0x00
#define HMC5883L_BIAS_POSITIVE      0x01
#define HMC5883L_BIAS_NEGATIVE      0x02

#define HMC5883L_CRB_GAIN_BIT       7
#define HMC5883L_CRB_GAIN_LENGTH    3

#define HMC5883L_GAIN_1370          0x00
#define HMC5883L_GAIN_1090          0x01
#define HMC5883L_GAIN_820           0x02
#define HMC5883L_GAIN_660           0x03
#define HMC5883L_GAIN_440           0x04
#define HMC5883L_GAIN_390           0x05
#define HMC5883L_GAIN_330           0x06
#define HMC5883L_GAIN_220           0x07

#define HMC5883L_MODEREG_BIT        1
#define HMC5883L_MODEREG_LENGTH     2

#define HMC5883L_MODE_CONTINUOUS    0x00
#define HMC5883L_MODE_SINGLE        0x01
#define HMC5883L_MODE_IDLE          0x02

#define HMC5883L_STATUS_LOCK_BIT    1
#define HMC5883L_STATUS_READY_BIT   0

#define HMC5883L_ST_GAIN            HMC5883L_GAIN_440  // Gain value during self-test
#define HMC5883L_ST_GAIN_NBR        440
#define HMC5883L_ST_ERROR           0.2                // Max error
#define HMC5883L_ST_DELAY_MS        250                // delay in millisec during self test */
#define HMC5883L_ST_X_NORM          (int32_t)(1.16 * HMC5883L_ST_GAIN_NBR)
#define HMC5883L_ST_X_MIN           (int32_t)(HMC5883L_ST_X_NORM - (HMC5883L_ST_X_NORM * HMC5883L_ST_ERROR))
#define HMC5883L_ST_X_MAX           (int32_t)(HMC5883L_ST_X_NORM + (HMC5883L_ST_X_NORM * HMC5883L_ST_ERROR))
#define HMC5883L_ST_Y_NORM          (int32_t)(1.16 * HMC5883L_ST_GAIN_NBR)
#define HMC5883L_ST_Y_MIN           (int32_t)(HMC5883L_ST_Y_NORM - (HMC5883L_ST_Y_NORM * HMC5883L_ST_ERROR))
#define HMC5883L_ST_Y_MAX           (int32_t)(HMC5883L_ST_Y_NORM + (HMC5883L_ST_Y_NORM * HMC5883L_ST_ERROR))
#define HMC5883L_ST_Z_NORM          (int32_t)(1.08 * HMC5883L_ST_GAIN_NBR)
#define HMC5883L_ST_Z_MIN           (int32_t)(HMC5883L_ST_Z_NORM - (HMC5883L_ST_Z_NORM * HMC5883L_ST_ERROR))
#define HMC5883L_ST_Z_MAX           (int32_t)(HMC5883L_ST_Z_NORM + (HMC5883L_ST_Z_NORM * HMC5883L_ST_ERROR))

void hmc5883lInit(I2C_Dev *i2cPort);
bool hmc5883lTestConnection();
bool hmc5883lSelfTest();
bool hmc5883lEvaluateSelfTest(int16_t min, int16_t max, int16_t value, char *string);

// CONFIG_A register
uint8_t hmc5883lGetSampleAveraging();
void hmc5883lSetSampleAveraging(uint8_t averaging);
uint8_t hmc5883lGetDataRate();
void hmc5883lSetDataRate(uint8_t rate);
uint8_t hmc5883lGetMeasurementBias();
void hmc5883lSetMeasurementBias(uint8_t bias);

// CONFIG_B register
uint8_t hmc5883lGetGain();
void hmc5883lSetGain(uint8_t gain);

// MODE register
uint8_t hmc5883lGetMode();
void hmc5883lSetMode(uint8_t mode);

// DATA* registers
void hmc5883lGetHeading(int16_t *x, int16_t *y, int16_t *z);
int16_t hmc5883lGetHeadingX();
int16_t hmc5883lGetHeadingY();
int16_t hmc5883lGetHeadingZ();

// STATUS register
bool hmc5883lGetLockStatus();
bool hmc5883lGetReadyStatus();

// ID_* registers
uint8_t hmc5883lGetIDA();
uint8_t hmc5883lGetIDB();
uint8_t hmc5883lGetIDC();

#endif /* HMC5883L_H_ */
