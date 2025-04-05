/*******************************************************************************
*1	File Name				:	SCApp_main.h
*	Copyright				:
*	Module Name				:	SCApp
*
*	CPU						:	None
*	RTOS					:	None
*
*	Creat Date				:	2013-11-23
*	Author/Corporation		:	GongYongjian/Andon,.Ltd
*
*	Abstract Description	:	
*		
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
*2	Multi-Include-Prevent Section
*******************************************************************************/
#ifndef SCAPP_MAIN_H
#define SCAPP_MAIN_H

/*******************************************************************************
*3	Debug Switch Section
*******************************************************************************/

/*******************************************************************************
*4	Include File Section
*******************************************************************************/


/*******************************************************************************
*5	Macro Define Section
*******************************************************************************/
#define SCAPP_TASK_ID			0x0d	//SCAPP任务ID

#define SCAPP_INIT_EVT       	        0x0001	//INIT事件序号	
#define SCAPP_UART_PRASE_EVT       	0x0002	// scale communication event		
#define SCAPP_CONFIG_EVT 	        0x0004	
#define SCAPP_DAC_EVT                   0x0008


#define SCAPP_TASK_ENTER_POWER_SAVING 	(osal_pwrmgr_task_state(SCAPP_TASK_ID,PWRMGR_CONSERVE))
#define SCAPP_TASK_EXIT_POWER_SAVING 	(osal_pwrmgr_task_state(SCAPP_TASK_ID,PWRMGR_HOLD))

/*******************************************************************************
*6	Struct Define Section
*******************************************************************************/  

/*******************************************************************************
*7	Global Variable Declare Section
*******************************************************************************/
extern unsigned char SCAPP_TaskID;

/*******************************************************************************
*8	Prototype Declare Section
*******************************************************************************/
extern void SCAPP_Init(unsigned char task_id);
extern unsigned short SCAPP_ProcessEvent( unsigned char task_id, unsigned short events );

#endif

/******************************** End of file *********************************/
