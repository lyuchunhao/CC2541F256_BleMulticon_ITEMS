/******************************************************************************

 @file  hal_i2c.h

 @brief HAL I2C API for the CC2541ST. It implements the I2C master only.

 Group: WCS, BTS
 Target Device: CC2540, CC2541

 ******************************************************************************
 
 Copyright (c) 2012-2016, Texas Instruments Incorporated
 All rights reserved.

 IMPORTANT: Your use of this Software is limited to those specific rights
 granted under the terms of a software license agreement between the user
 who downloaded the software, his/her employer (which must be your employer)
 and Texas Instruments Incorporated (the "License"). You may not use this
 Software unless you agree to abide by the terms of the License. The License
 limits your use, and you acknowledge, that the Software may not be modified,
 copied or distributed unless embedded on a Texas Instruments microcontroller
 or used solely and exclusively in conjunction with a Texas Instruments radio
 frequency transceiver, which is integrated into your product. Other than for
 the foregoing purpose, you may not use, reproduce, copy, prepare derivative
 works of, modify, distribute, perform, display or sell this Software and/or
 its documentation for any purpose.

 YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
 PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
 INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
 NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
 TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
 NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
 LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
 INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
 OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
 OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
 (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

 Should you have any questions regarding your right to use this Software,
 contact Texas Instruments Incorporated at www.TI.com.

 ******************************************************************************
 Release Name: ble_sdk_1.4.2.2
 Release Date: 2016-06-09 06:57:09
 *****************************************************************************/

#ifndef HAL_I2C_H
#define HAL_I2C_H

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */
#include "comdef.h"

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */
#define HAL_I2C_SLAVE_ADDR_DEF           0x41


#define HTPA_DEV_ADDR        (0x1B >> 1)
#define MCP4728_DEV_ADDR     0x60                       //MCP4728Ä¬ÈÏ´Ó»úµØÖ· 0xC0 = 1100 000(RW)
#define LDAC_HIGH()          st( P0 |= ( BV(5) ); )       
#define LDAC_LOW()           st( P0 &= ~( BV(5) ); )
#define LDAC_PIN             P1_5
/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
 * ------------------------------------------------------------------------------------------------
 */
typedef enum
{
  i2cClock_123KHZ = 0x00,
  i2cClock_144KHZ = 0x01,
  i2cClock_165KHZ = 0x02,
  i2cClock_197KHZ = 0x03,
  i2cClock_33KHZ  = 0x80,
  i2cClock_267KHZ = 0x81,
  i2cClock_533KHZ = 0x82
} i2cClock_t;

typedef enum
{
  MCP4728_Gx_1 = 0x00,
  MCP4728_Gx_2 = 0x01  //2±¶ÔöÒæ
  
}MCP4728_Gx;

/* ------------------------------------------------------------------------------------------------
 *                                       Global Functions
 * ------------------------------------------------------------------------------------------------
 */
void     HalI2CInit(uint8 address, i2cClock_t clockRate);
uint8    HalI2CRead(uint8 len, uint8 *pBuf);
uint8    HalI2CWrite(uint8 len, uint8 *pBuf);
void     HalI2CDisable(void);
void     HalI2CEnable(void);

void Delay_Us(uint16 u16time);
void MCP4728_Init(uint8 address, i2cClock_t clockRate);
void MCP4728_DACSingleWriteByVDD(uint8 u8Channel, uint16 u16DataToWrite);
void MCP4728_DACSingleWrite(uint8 u8Channel, uint8 u8Gx, uint16 u16DataToWrite);
void MCP4728_DACRegisterRead(uint8 len, uint8 *pBuf);
void MCP4728_DACContiWrite(uint16 u16DataToWrite[4]);

#endif
/**************************************************************************************************
 */
