/******************************************************************************

 @file  hal_i2c.c

 @brief This module defines the HAL I2C API for the CC2541ST. It implements the
        I2C master.

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
 PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */
#include "hal_assert.h"
#include "hal_board_cfg.h"
#include "mcp4728_i2c.h"

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */
#define I2C_ENS1            BV(6)
#define I2C_STA             BV(5)
#define I2C_STO             BV(4)
#define I2C_SI              BV(3)
#define I2C_AA              BV(2)
#define I2C_MST_RD_BIT      BV(0)  // Master RD/WRn bit to be OR'ed with Slave address.

#define I2C_CLOCK_MASK      0x83

#define I2C_PXIFG           P2IFG
#define I2C_IF              P2IF
#define I2C_IE              BV(1)

/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
 * ------------------------------------------------------------------------------------------------
 */

typedef enum
{
  // HAL_I2C_MASTER mode statuses.
  mstStarted   = 0x08,
  mstRepStart  = 0x10,
  mstAddrAckW  = 0x18,
  mstAddrNackW = 0x20,
  mstDataAckW  = 0x28,
  mstDataNackW = 0x30,
  mstLostArb   = 0x38,
  mstAddrAckR  = 0x40,
  mstAddrNackR = 0x48,
  mstDataAckR  = 0x50,
  mstDataNackR = 0x58,
} i2cStatus_t;

/* ------------------------------------------------------------------------------------------------
 *                                           Macros
 * ------------------------------------------------------------------------------------------------
 */

#define I2C_WRAPPER_DISABLE() st( I2CWC    =     0x00;              )
#define I2C_CLOCK_RATE(x)     st( I2CCFG  &=    ~I2C_CLOCK_MASK;    \
                                  I2CCFG  |=     x;                 )
#define I2C_SET_NACK()        st( I2CCFG &= ~I2C_AA; )
#define I2C_SET_ACK()         st( I2CCFG |=  I2C_AA; )

// Enable I2C bus
#define I2C_ENABLE()          st( I2CCFG |= (I2C_ENS1); )
//#define I2C_ENABLE()          st( I2CCFG |= (I2C_ENS1 | I2C_AA); )
#define I2C_DISABLE()         st( I2CCFG &= ~(I2C_ENS1); )

// Must clear SI before setting STA and then STA must be manually cleared.
#define I2C_STRT() st (             \
  I2CCFG &= ~I2C_SI;                \
  I2CCFG |= I2C_STA;                \
  while ((I2CCFG & I2C_SI) == 0);   \
  I2CCFG &= ~I2C_STA; \
)

#if 0
// Must set STO before clearing SI.
#define I2C_STOP() st (             \
  I2CCFG |= I2C_STO;                \
  I2CCFG &= ~I2C_SI;                \
  while ((I2CCFG & I2C_STO) != 0);  \
  I2CCFG &=  ~I2C_ENS1;            \
)
#else
#define I2C_STOP() st (             \
  I2CCFG |= I2C_STO;                \
  I2CCFG &= ~I2C_SI;                \
  while ((I2CCFG & I2C_STO) != 0);  \
)
#endif

// Stop clock-stretching and then read when it arrives.
#define I2C_READ(_X_) st (          \
  I2CCFG &= ~I2C_SI;                \
  while ((I2CCFG & I2C_SI) == 0);   \
  (_X_) = I2CDATA;                  \
)

// First write new data and then stop clock-stretching.
#define I2C_WRITE(_X_) st (         \
  I2CDATA = (_X_);                  \
  I2CCFG &= ~I2C_SI;                \
  while ((I2CCFG & I2C_SI) == 0);   \
)


/* ------------------------------------------------------------------------------------------------
 *                                       Local Variables
 * ------------------------------------------------------------------------------------------------
 */
#define ERROR_I2CBUS_ERROR		0xe7	//I2C���ߴ���

static uint8 i2cAddr;  // Target Slave address pre-shifted up by one leaving RD/WRn LSB as zero.
unsigned char ErrorCode;
unsigned char I2CSend_Failure ;//ȫ�ֱ���
    
#if 0
/**************************************************************************************************
 * @fn          i2cMstStrt
 *
 * @brief       Attempt to send an I2C bus START and Slave Address as an I2C bus Master.
 *
 * input parameters
 *
 * @param       RD_WRn - The LSB of the Slave Address as Read/~Write.
 *
 * @return      The I2C status of the START request or of the Slave Address Ack.
 */
static uint8 i2cMstStrt(uint8 RD_WRn)
{
  I2C_STRT();

  if (I2CSTAT == mstStarted) /* A start condition has been transmitted */
  {
    I2C_WRITE(i2cAddr | RD_WRn);
  }

  return I2CSTAT;
}

/**************************************************************************************************
 * @fn          HalI2CInit
 *
 * @brief       Initialize the I2C bus as a Master.
 *
 * input parameters
 *
 * @param       address - I2C slave address.
 * @param       clockRate - I2C clock rate.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 */
void HalI2CInit(uint8 address, i2cClock_t clockRate)
{
  i2cAddr = address << 1;

  I2C_WRAPPER_DISABLE();
  I2CADDR = 0; // no multi master support at this time
  I2C_CLOCK_RATE(clockRate);
  I2C_ENABLE();
}

/**************************************************************************************************
 * @fn          HalI2CRead
 *
 * @brief       Read from the I2C bus as a Master.
 *
 * input parameters
 *
 * @param       len - Number of bytes to read.
 * @param       pBuf - Pointer to the data buffer to put read bytes.
 *
 * output parameters
 *
 * None.
 *
 * @return      The number of bytes successfully read.
 */
uint8 HalI2CRead(uint8 len, uint8 *pBuf)
{
  uint8 cnt = 0;

  if (i2cMstStrt(I2C_MST_RD_BIT) != mstAddrAckR)
  {
    len = 0;
  }

  // All bytes are ACK'd except for the last one which is NACK'd. If only
  // 1 byte is being read, a single NACK will be sent. Thus, we only want
  // to enable ACK if more than 1 byte is going to be read.
  if (len > 1)
  {
    I2C_SET_ACK();
  }

  while (len > 0)
  {
    // slave devices require NACK to be sent after reading last byte
    if (len == 1)
    {
      I2C_SET_NACK();
    }

    // read a byte from the I2C interface
    I2C_READ(*pBuf++);
    cnt++;
    len--;

    if (I2CSTAT != mstDataAckR)
    {
      if (I2CSTAT != mstDataNackR)
      {
        // something went wrong, so don't count last byte
        cnt--;
      }
      break;
    }
  }
  I2C_STOP();

  return cnt;
}
uint8 HalI2CReadNoStart(uint8 len, uint8 *pBuf)
{
  uint8 cnt = 0;

  //if (i2cMstStrt(I2C_MST_RD_BIT) != mstAddrAckR)
  //{
    //len = 0;
  //}

  // All bytes are ACK'd except for the last one which is NACK'd. If only
  // 1 byte is being read, a single NACK will be sent. Thus, we only want
  // to enable ACK if more than 1 byte is going to be read.
  if (len > 1)
  {
    I2C_SET_ACK();
  }

  while (len > 0)
  {
    // slave devices require NACK to be sent after reading last byte
    if (len == 1)
    {
      I2C_SET_NACK();
    }

    // read a byte from the I2C interface
    I2C_READ(*pBuf++);
    cnt++;
    len--;

    if (I2CSTAT != mstDataAckR)
    {
      if (I2CSTAT != mstDataNackR)
      {
        // something went wrong, so don't count last byte
        cnt--;
      }
      break;
    }
  }
  I2C_STOP();

  return cnt;
}
/**************************************************************************************************
 * @fn          HalI2CWrite
 *
 * @brief       Write to the I2C bus as a Master.
 *
 * input parameters
 *
 * @param       len - Number of bytes to write.
 * @param       pBuf - Pointer to the data buffer to write.
 *
 * output parameters
 *
 * None.
 *
 * @return      The number of bytes successfully written.
 */
uint8 u8CopyData[4];
uint8 HalI2CWrite(uint8 len, uint8 *pBuf)
{
  if (i2cMstStrt(0) != mstAddrAckW)
  {
    len = 0;
  }

  for (uint8 cnt = 0; cnt < len; cnt++)
  {
    u8CopyData[cnt] = *pBuf;
    I2C_WRITE(*pBuf++);

    if (I2CSTAT != mstDataAckW)
    {
      if (I2CSTAT == mstDataNackW)
      {
        len = cnt + 1;
      }
      else
      {
        len = cnt;
      }
      break;
    }
  }

  I2C_STOP();

  return len;
}

#else
/**************************************************************************************************
 * @fn         I2C_Repeat
 *
 * @brief       Restart I2C when I2C transmit occur error
 *
 * input parameters
 *
 * @param       ercntRD_ - the count of restart.
 *
 * output parameters
 *
 * @return      .
 *////////////////////////////////////////////////////////////////////////////////////////
unsigned char I2C_Repeat(unsigned char ercnt)
{
	unsigned char temp;
	temp=ercnt;
	I2C_STOP();
	HalI2CInit(MCP4728_DEV_ADDR, i2cClock_267KHZ);//������267KHz//i2_reset();
	temp++;
	
	if(temp>=10)
	{
		temp=10;
		ErrorCode = ERROR_I2CBUS_ERROR;//run_mode=MODE_er7;
		//EI();
		return temp;
	}
		
	else
	{
		//EI();
		//i2_delay_ms(10);
		return temp;
	}
}

/**************************************************************************************************
 * @fn          i2cMstStrt
 *
 * @brief       Attempt to send an I2C bus START and Slave Address as an I2C bus Master.
 *
 * input parameters
 *
 * @param       RD_WRn - The LSB of the Slave Address as Read/~Write.
 *
 * output parameters
 *
 * None.
 *
 * @return      The I2C status of the START request or of the Slave Address Ack.
 */
static uint8 i2cMstStrt(uint8 RD_WRn)//����������ʼ�źźʹӻ���ַ
{
  /////////////////
  //I2C_STOP();
  HalI2CInit(MCP4728_DEV_ADDR, i2cClock_267KHZ);//������267KHz
  ///////����������Ϊ�˸�λI2Cģ��//////////
  
  I2C_STRT();	//������ʼ�ź�
  
//  I2CCFG &= ~I2C_SI;              
//  I2CCFG |= I2C_STA;    
//  
//  while ((I2CCFG & I2C_SI) == 0);  
//  I2CCFG &= ~I2C_STA;  
  
  if (I2CSTAT == mstStarted)
  {
	I2C_WRITE(i2cAddr | RD_WRn);	//���ʹӻ���ַ������λ
  }
  return I2CSTAT;
}

/**************************************************************************************************
 * @fn          HalI2CInit
 *
 * @brief       Initialize the I2C bus as a Master.
 *
 * input parameters
 *
 * @param       address - I2C slave address.
 * @param       clockRate - I2C clock rate.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 */
void HalI2CInit(uint8 address, i2cClock_t clockRate)//I2C��Ϊ��������  ��ʼ��
{
  i2cAddr = address << 1;

  I2C_WRAPPER_DISABLE();
  I2C_CLOCK_RATE(clockRate);
  I2C_ENABLE();
}

/**************************************************************************************************
 * @fn          HalI2CRead
 *
 * @brief       Read from the I2C bus as a Master.
 *
 * input parameters
 *
 * @param       len - Number of bytes to read.
 * @param       pBuf - Pointer to the data buffer to put read bytes.
 *
 * output parameters
 *
 * None.
 *
 * @return      The number of bytes successfully read.
 */
uint8 HalI2CRead(uint8 len, uint8 *pBuf)
{
  if (i2cMstStrt(I2C_MST_RD_BIT) != mstAddrAckR)
  {
	len = 0;
  }

  for (uint8 cnt = 0; cnt < len; cnt++)
  {
	I2C_READ(*pBuf++);

	if (I2CSTAT != mstDataAckR)
	{
	  if (I2CSTAT == mstDataNackR)
	  {
		len = cnt + 1;
	  }
	  else
	  {
		len = cnt;
	  }
	  break;
	}
  }

  I2C_STOP();
  return len;
}

/**************************************************************************************************
 * @fn          HalI2CWrite
 *
 * @brief       Write to the I2C bus as a Master.
 *
 * input parameters
 *
 * @param       len - Number of bytes to write.
 * @param       pBuf - Pointer to the data buffer to write.
 *
 * output parameters
 *
 * None.
 *
 * @return      The number of bytes successfully written.
 */
uint8 HalI2CWrite(uint8 len, uint8 *pBuf)
{
	unsigned char Err_cnt;//�������Դ�������
	I2CSend_Failure = 0;	
	
i2wr_repeat:
	if (i2cMstStrt(0) != mstAddrAckW)	//���ʹӻ���ַ�յ�NCK
	{
		Err_cnt = I2C_Repeat(Err_cnt);   //��ʼ��I2Cģ��
		if(ErrorCode != ERROR_I2CBUS_ERROR )		 //���Դ���δ�� ���·��ʹӻ���ַ
			goto i2wr_repeat;
		else
			len = 0;		  //����10������ʧ�ܣ�������ȷ�����ֽ���Ϊ0
	}

	for (uint8 cnt = 0; cnt < len; cnt++)
	{
		I2C_WRITE(*pBuf++);				//��������
	
		if (I2CSTAT != mstDataAckW)  	//���ݷ���ʧ��
		{
			Err_cnt = I2C_Repeat(Err_cnt);//��ʼ��I2Cģ��
			if(ErrorCode != ERROR_I2CBUS_ERROR)
				goto i2wr_repeat;//���Դ���δ�� ���·��ʹӻ���ַ
			else
			{
				if (I2CSTAT == mstDataNackW)
				{
					len = cnt + 1;
				}
				else
				{
					len = cnt;
				}
			}
		  break;
		}
	}

	I2C_STOP();
	return len;
}

#endif

/**************************************************************************************************
 * @fn          HalI2CDisable
 *
 * @brief       Places the I2C bus in inactive mode
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 */
void HalI2CDisable(void)
{
  I2C_DISABLE();
}
/**************************************************************************************************
 * @fn          HalI2CEnable
 *
 * @brief       Places the I2C bus in inactive mode
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 */
void HalI2CEnable(void)
{
  I2C_ENABLE();
}


void Delay_Us(uint16 u16time)
{
  uint8 u8Cnt;
  for(u8Cnt = 0; u8Cnt < 32; u8Cnt ++)
  {
    asm("nop");
  }
}

/* MCP4728��ʼ������ */
void MCP4728_Init(uint8 address, i2cClock_t clockRate)
{
  //LDAC�˿ڹܽų�ʼ��P1_5��ͨ���
  P1DIR |= BV(5);
  P1SEL &= ~BV(5);
  
  LDAC_PIN = 0;
  
  //MCP��I2C��ʼ��:����I2Cģ��,��¼�´ӻ���ַ,����Ƶ��
  HalI2CInit(address, clockRate);
}
/* 
 * ����дDAC����Ĵ�����EEPROM:ָ������ͨ����д��EEPROM(DAC1��DAC0ָ��ͨ��)
 * ��һ�ֽڣ� (S)  1 1 0 0 A2 A1 A0 0 (ASK) 
 *           ���� ������ַA2-A1Ĭ��000 ���һλΪ��дRWλ��0Ϊд                   
 * �ڶ��ֽڣ�C2 C1 C0 W1 W0 DAC1 DAC0 UDAC (ASK)   [DAC1DAC0��00ͨ��A 01ͨ��B 10-C 11-D]
 *           0  1  0  1  1  ͨ��ѡ��  �Ƿ����     [UDAC��DAC����λ 0-���� 1-������]
 * �����ֽڣ�Vref PD1 PD0 Gx D11 D10 D9 D8 (ASK)   [Vref��0=Vdd 1=�ڲ���׼2.048]
 *           �ο� �ض�   ����  ����12bit           [Gx=0 ����Ϊ1 Gx=1 ����Ϊ2]
 * �����ֽڣ�D7 D6 D5 D4 D3 D2 D1 D0 (ASK) (P)     [PD1PD0��00 ����ģʽ 01 1Kŷ���ؽӵ� 10 100K 11 500K�ӵ�]
 *           ����λ                       ֹͣλ    
 *
 *������u8Gx ����ѡ�� 0-1���������0-2048mv 1-2���������0-4096mv
 *      u8Channel ͨ��ѡ�� 0123-ABCD
 *      u16DataToWrite ��Ҫ����ĵ�ѹֵmv u8Gx = 0ʱȡֵ��ΧΪ[0,2048] u8Gx=1ʱ[0-4096]
 *                     ���ȡֵ������Χ�������Ϊ0����
*/
//uint8 u8Write[4] = {0};
//uint16 u16TmpDataToWrite = 0;
void MCP4728_DACSingleWrite(uint8 u8Channel, uint8 u8Gx, uint16 u16DataToWrite)
{
  uint8 u8Write[4] = {0};
  uint16 u16TmpDataToWrite = 0;
  
  u16TmpDataToWrite = u16DataToWrite;
  if(u8Gx == MCP4728_Gx_2)
  {
    //�������������̣������Ϊ0
    if(u16TmpDataToWrite >= 4096)
    {
      u16TmpDataToWrite = 0;
    }
    
    //��4bitΪ 1001 1�ڲ���׼��ѹ 00 ���� 1����=2 ���Ϊ0-4096mv
    u16TmpDataToWrite &= 0x0FFF;
    u16TmpDataToWrite |= 0x9000;
  }
  else
  {
    //�������������̣������Ϊ0
    if(u16TmpDataToWrite >= 2048)
    {
      u16TmpDataToWrite = 0;
    }
    
    u16TmpDataToWrite *= 2;

    //��4bitΪ 1000 1�ڲ���׼��ѹ 00 ���� 0����=2 ���Ϊ0-2048mv
    u16TmpDataToWrite &= 0x0FFF;
    u16TmpDataToWrite |= 0x8000;
  }
  
  //UDAC = 0�õ�ACK��������
  u8Write[0] = 0x58 + (u8Channel << 1);
  u8Write[1] = (uint8)(u16TmpDataToWrite >> 8);
  u8Write[2] = (uint8)u16TmpDataToWrite;
  HalI2CWrite(3, u8Write);
  
  Delay_Us(10);
}
/* �ο���ѹΪVDD = 3.3V */
void MCP4728_DACSingleWriteByVDD(uint8 u8Channel, uint16 u16DataToWrite)
{
  uint8 u8Write[4] = {0};
  uint16 u16TmpDataToWrite = 0;
  double dbTmpDataToWrite = 1.2412;         //4096/3300 = 1.2412
  
  u16TmpDataToWrite = u16DataToWrite;

  //�������������̣������Ϊ0
  if(u16TmpDataToWrite >= 3300)
  {
    u16TmpDataToWrite = 0;
  }
  
  dbTmpDataToWrite *= u16DataToWrite;
  u16TmpDataToWrite = (uint16)dbTmpDataToWrite;
  
  //��4bitΪ 100x 0�ο���ѹΪVDD 00 ���� x�����ʱ��Ч ���Ϊ0-4096mv
  u16TmpDataToWrite &= 0x0FFF;
  //u16TmpDataToWrite |= 0x9000;
  
  //UDAC = 0�õ�ACK��������
  u8Write[0] = 0x58 + (u8Channel << 1);
  u8Write[1] = (uint8)(u16TmpDataToWrite >> 8);
  u8Write[2] = (uint8)u16TmpDataToWrite;
  HalI2CWrite(3, u8Write);
  
  Delay_Us(10);
}
/* ���дDAC����Ĵ��� */
void MCP4728_DACMultiWrite(uint8 u8Channel, uint16 u16DataToWrite)
{
  
}
/* ����дDAC����Ĵ���������ֱ�ΪA-Dͨ����ֵ����δ��ͨCDͨ������ */
void MCP4728_DACContiWrite(uint16 u16DataToWrite[4])
{
  uint8 u8Write[16] = {0x50,0x91,0x04,0x91,0x04,0x92,0x55,0x92,0x55};
  uint16 u16TmpDataToWrite = 0;

#if 1  
  //ͨ��A
  u16TmpDataToWrite = u16DataToWrite[0];
  //�������������̣������Ϊ0
  if(u16TmpDataToWrite >= 4096)
  {
    u16TmpDataToWrite = 0;
  }
    
  //��4bitΪ 1001 1�ڲ���׼��ѹ 00 ���� 1����=2 ���Ϊ0-4096mv
  u16TmpDataToWrite &= 0x0FFF;
  u16TmpDataToWrite |= 0x9000;
  
  //UDAC = 0�õ�ACK��������
  u8Write[0] = 0x50 + (0 << 1);
  u8Write[1] = (uint8)(u16TmpDataToWrite >> 8);
  u8Write[2] = (uint8)u16TmpDataToWrite;
  
  //ͨ��B
  u16TmpDataToWrite = u16DataToWrite[1];
  //�������������̣������Ϊ0
  if(u16TmpDataToWrite >= 4096)
  {
    u16TmpDataToWrite = 0;
  }
    
  //��4bitΪ 1001 1�ڲ���׼��ѹ 00 ���� 1����=2 ���Ϊ0-4096mv
  u16TmpDataToWrite &= 0x0FFF;
  u16TmpDataToWrite |= 0x9000;
  
  //UDAC = 0�õ�ACK��������
  u8Write[3] = (uint8)(u16TmpDataToWrite >> 8);
  u8Write[4] = (uint8)u16TmpDataToWrite;
  
  //ͨ��C
  u16TmpDataToWrite = u16DataToWrite[2];
  //�������������̣������Ϊ0
  if(u16TmpDataToWrite >= 4096)
  {
    u16TmpDataToWrite = 0;
  }
    
  //��4bitΪ 1001 1�ڲ���׼��ѹ 00 ���� 1����=2 ���Ϊ0-4096mv
  u16TmpDataToWrite &= 0x0FFF;
  u16TmpDataToWrite |= 0x9000;
  
  //UDAC = 0�õ�ACK��������
  u8Write[5] = (uint8)(u16TmpDataToWrite >> 8);
  u8Write[6] = (uint8)u16TmpDataToWrite;
  
  //ͨ��D
  u16TmpDataToWrite = u16DataToWrite[3];
  //�������������̣������Ϊ0
  if(u16TmpDataToWrite >= 4096)
  {
    u16TmpDataToWrite = 0;
  }
    
  //��4bitΪ 1001 1�ڲ���׼��ѹ 00 ���� 1����=2 ���Ϊ0-4096mv
  u16TmpDataToWrite &= 0x0FFF;
  u16TmpDataToWrite |= 0x9000;
  
  //UDAC = 0�õ�ACK��������
  u8Write[7] = (uint8)(u16TmpDataToWrite >> 8);
  u8Write[8] = (uint8)u16TmpDataToWrite;
#endif
  
  HalI2CWrite(9, u8Write);
  
  
}
/* ��ͨ��DAC�Ĵ�����EEPROM�� */
 void MCP4728_DACRegisterRead(uint8 len, uint8 *pBuf)
 {
   
   HalI2CRead(len, pBuf);
#if 0
   HalI2CRead(3, pBuf + 3);
   HalI2CRead(3, pBuf + 6);
   HalI2CRead(3, pBuf + 9);
   HalI2CRead(3, pBuf + 12);
   HalI2CRead(3, pBuf + 15);
   HalI2CRead(3, pBuf + 18);
   HalI2CRead(3, pBuf + 21);
#endif
 }

/*********************************************************************
*********************************************************************/
