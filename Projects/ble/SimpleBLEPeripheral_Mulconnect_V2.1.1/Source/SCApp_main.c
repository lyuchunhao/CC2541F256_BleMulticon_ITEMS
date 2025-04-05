/*******************************************************************************
*1	File Name				:	SCApp_main.c
*	Copyright				:
*	Module Name				:	SCApp
*
*	CPU						:	CC2541
*	RTOS					:	None
*
*	Creat Date				:	2013-11-23
*	Author/Corporation		:	GongYongjian/Andon,.Ltd
*
*	Abstract Description	:	
*		
*---Revision History------------------------------------------------------------
*	No.			:	1
*	Version		:	V0.1
*	Date		:	2013-11-23
*	Revised By	:	GongYongjian
*	Item		:
*	Description	:	First edition
*
*******************************************************************************/

/*******************************************************************************
*2	Debug Switch Section
*******************************************************************************/

/*******************************************************************************
*3	Include File Section
*******************************************************************************/
#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"

#include "gatt.h"

#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simpleGATTprofile.h"

#if defined( CC2540_MINIDK )
  #include "simplekeys.h"
#endif

#include "peripheral.h"

#include "gapbondmgr.h"

#include "simpleBLEPeripheral.h"

#include "SerialApp.h"

#if defined FEATURE_OAD
  #include "oad.h"
  #include "oad_target.h"
#endif

#include "mcp4728_i2c.h"
#include "SCApp_main.h"
#include "simpleBLEUser.h"

/*******************************************************************************
*4	Macro Define Section
*******************************************************************************/

/*******************************************************************************
*5	Struct Define Section
*******************************************************************************/

/*******************************************************************************
*6	Prototype Declare Section
*******************************************************************************/


/*******************************************************************************
*7	Global Variable Define Section
*******************************************************************************/
unsigned char SCAPP_TaskID;

/*******************************************************************************
*8	File Static Variable Define Section
*******************************************************************************/

/*******************************************************************************
*9	Function Define Section
*******************************************************************************/


/*==============================================================================
=	Function Name		:	SCAPP_Init
=	Creat Date			:	2012-09-21
=	Author/Corporation	:	GongYongjian/Andon,.Ltd
=	
=	Description			:	SCAPP�¼���ʼ������
=							
=
=	Parameter			:	task_id
=
=	Return Code			:	None
=
=	Global Variable		:	None
=
=-------------------------------------------------------------------------------
=	Revision History
=	No.			:	V0.1
=	Date		:	2012-09-21
=	Revised By	:	GongYongjian/Andon,.Ltd
=	Item		:
=	Description	:	First edition
=
==============================================================================*/
void SCAPP_Init(unsigned char task_id)
{
	SCAPP_TaskID = task_id;
        
        //serial port initialization
        SerialApp_Init(task_id);
        sbpSerialAppWrite("SimpleBLEPeripheral_Init Start init.\r\n", 38);
        
        
        //MCP4728��ʼ��
        //MCP4728_Init(MCP4728_DEV_ADDR, i2cClock_267KHZ); 

        //����ʱ�����ʱ����Ϊ8M
        //PWM��Ƶ1k-30k��P1_3 P2_0(����1)
        PwmHighFreInit();
        PwmHighFreSetCycle(30);
        
        //PWM��Ƶ1-30Hz:P1_0 P1_1 (����2)
        PwmLowFrequencyInit();

}


/*==============================================================================
=	Function Name		:	SCAPP_ProcessEvent
=	Creat Date			:	2012-09-21
=	Author/Corporation	:	GongYongjian/Andon,.Ltd
=	
=	Description			:	SCAPP�¼�������
=							
=
=	Parameter			:	None
=
=	Return Code			:	None
=
=	Global Variable		:	None
=
=-------------------------------------------------------------------------------
=	Revision History
=	No.			:	V0.1
=	Date		:	2012-09-21
=	Revised By	:	GongYongjian/Andon,.Ltd
=	Item		:
=	Description	:	First edition
=
==============================================================================*/
uint16 SCAPP_ProcessEvent( uint8 task_id, uint16 events )
{
        VOID task_id;
        uint8  u8Return = 0;            //Temporary parameter
        uint8  u8PwmCycle = 1;          //Temporary Pwm parameter
	
  
        if (events & SCAPP_INIT_EVT)
	{


		return ( events ^ SCAPP_INIT_EVT );
	}
	
	if (events & SCAPP_UART_PRASE_EVT)
	{
                //��������>=3Bytes�����
                u8Return = sbpUartDataParse();
                if(u8Return != 0)
                {
                  switch(g_Praseu8UartData[2])
                  {
                    case 0x2B://F5 05 2B 00 0F 00 0A
                              SerialPrintValue("PwmHighFre:", g_Praseu8UartData[4], 10);
                              SerialPrintString("KHz\n");
                              SerialPrintValue("PwmLowFre :", g_Praseu8UartData[6], 10);
                              SerialPrintString("Hz\n");
                              
                              u8PwmCycle = g_Praseu8UartData[4];
                              PwmHighFreSetCycle(u8PwmCycle);
                              u8PwmCycle = g_Praseu8UartData[6];
                              PwmLowFrequencySetCycle(u8PwmCycle, 10, 10, 10, PWM_SITUATION_ONE);
                              break;
                              
                    case 0x2A://DA����: F5 05 2A 00 0F 00 0A
                              SerialPrintValue("Mcp4728 set:", g_Praseu8UartData[4], 10);
                              SerialPrintValue("Mcp4728 set:", g_Praseu8UartData[6], 10);
                              SerialPrintString("\n");
                              
                              g_u16McpDACValue[0] = 256*g_Praseu8UartData[3] + g_Praseu8UartData[4];
                              g_u16McpDACValue[1] = 256*g_Praseu8UartData[3] + g_Praseu8UartData[4];
                              g_u16McpDACValue[2] = 256*g_Praseu8UartData[5] + g_Praseu8UartData[6];
                              g_u16McpDACValue[3] = 256*g_Praseu8UartData[5] + g_Praseu8UartData[6];
                              
                                    SerialPrintValue("Channel-A: ", g_u16McpDACValue[0], 10);
                              SerialPrintString("\n");
                                    SerialPrintValue("Channel-B: ", g_u16McpDACValue[1], 10);
                              SerialPrintString("\n");
                                    SerialPrintValue("Channel-C: ", g_u16McpDACValue[2], 10);
                              SerialPrintString("\n");
                                    SerialPrintValue("Channel-D: ", g_u16McpDACValue[3], 10);
                              SerialPrintString("\n");
                              
                              MCP4728_DACContiWrite(g_u16McpDACValue);
                              break;
                              
                    default : //��ӡ�����ڲ鿴
                              sbpSerialAppWrite(g_Praseu8UartData, u8Return);
                              break;
                  }
                  
                }

		
		return ( events ^ SCAPP_UART_PRASE_EVT );
	}
	

	if (events & SCAPP_CONFIG_EVT)
	{

	
	        /*  
                  1. VDDΪ�ο���ѹ�Ĳ��ԡ����0.04v���ҡ�
                  2. 0ͨ�����[2�������ڲ��ο�2.048]4.096V�����0.01V��
                  3. LDAC_PIN = 0��MCP4728д�Ĵ��������DAֵ
                  4. ����DAC�Ĵ����Ĳ����ƺ����������㹻����ʱ�䣬����ڶ��β�����Ч
                  example��
                  MCP4728_DACSingleWriteByVDD(0, 3250);               [˵��1]
                  MCP4728_DACSingleWrite(0, MCP4728_Gx_2, 2000);      [˵��2]
                  MCP4728_DACRegisterRead(24, u8MCP4728ConfigData);   [���Ĵ�����������]
                */
                
                //�������DA
                LDAC_PIN = 0;
                   
                //��Ƶ              
                PwmHighFreSetCycle(g_u16FrequencyValue[0]); 
                //10Hz ����1ms ���10*20us = 200us ��ʼ10*20=200us
                PwmLowFrequencySetCycle(g_u16FrequencyValue[1], g_u16FrequencyValue[2], g_u16FrequencyValue[3], 10, PWM_SITUATION_TWO);

                osal_start_timerEx( SCAPP_TaskID, SCAPP_DAC_EVT, 50 );

                	
		return (events ^ SCAPP_CONFIG_EVT);
	}
        
        if (events & SCAPP_DAC_EVT)
	{
          
                      SerialPrintValue("Channel-A: ", g_u16McpDACValue[0], 10);
                SerialPrintString("\n");
                      SerialPrintValue("Channel-B: ", g_u16McpDACValue[1], 10);
                SerialPrintString("\n");
                      SerialPrintValue("Channel-C: ", g_u16McpDACValue[2], 10);
                SerialPrintString("\n");
                      SerialPrintValue("Channel-D: ", g_u16McpDACValue[3], 10);
                SerialPrintString("\n");
  
                MCP4728_DACContiWrite(g_u16McpDACValue);
                g_u8ConfigActFlag = 0;
                              
                return (events ^ SCAPP_DAC_EVT);
        }

        
        

	
	return ( events );
}


/******************************** End of file *********************************/