#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"

#include "hal_lcd.h"

#include "hal_uart.h"


#include "gatt.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "simpleGATTprofile.h"


#include "SerialApp.h"
#include "SCApp_main.h"
#include "simpleBLEPeripheral.h"

//static uint8 sendMsgTo_TaskID;
extern uint8 sbpGattWriteString(uint8 *pBuffer, uint16 length);

/*该函数将会在任务函数的初始化函数中调用*/
void SerialApp_Init( uint8 taskID )
{
  //调用uart初始化代码
  serialAppInitTransport();
  //记录任务函数的taskID，备用
  //sendMsgTo_TaskID = taskID;
}

/*uart初始化代码，配置串口的波特率、流控制等*/
void serialAppInitTransport( )
{
  halUARTCfg_t uartConfig;

  // configure UART
  uartConfig.configured           = TRUE;
  uartConfig.baudRate             = SBP_UART_BR;             //波特率
  uartConfig.flowControl          = SBP_UART_FC;             //流控制
  uartConfig.flowControlThreshold = SBP_UART_FC_THRESHOLD;   //流控制阈值，当开启flowControl时，该设置有效
  uartConfig.rx.maxBufSize        = SBP_UART_RX_BUF_SIZE;    //uart接收缓冲区大小
  uartConfig.tx.maxBufSize        = SBP_UART_TX_BUF_SIZE;    //uart发送缓冲区大小
  uartConfig.idleTimeout          = SBP_UART_IDLE_TIMEOUT;
  uartConfig.intEnable            = SBP_UART_INT_ENABLE;     //是否开启中断
  uartConfig.callBackFunc         = sbpSerialAppCallback;    //uart接收回调函数，在该函数中读取可用uart数据

  // start UART
  // Note: Assumes no issue opening UART port.
  (void)HalUARTOpen( SBP_UART_PORT, &uartConfig );

  return;
}

/*uart接收回调函数*/
uint16 numBytes;
void sbpSerialAppCallback(uint8 port, uint8 event)
{
  //static unsigned count=0;
  uint8  pktBuffer[SBP_UART_RX_BUF_SIZE];
  
  // unused input parameter; PC-Lint error 715.
  (void)event;
  
  //返回可读的字节
  if ( (numBytes = Hal_UART_RxBufLen(port)) > 0 )
  {
  	//读取全部有效的数据，这里可以一个一个读取，以解析特定的命令
	(void)HalUARTRead (port, pktBuffer, numBytes);
	//Ble 发送
	sbpSerialAppSendNoti(pktBuffer, numBytes);
        
        //osal_memcpy(g_u8UartData, pktBuffer, numBytes);
        for(uint16 Cnt = 0; Cnt < numBytes; Cnt ++)
        {
            g_u8UartData[g_u8UartWritePtr] = pktBuffer[Cnt];
            g_u8UartWritePtr++;
            
            if(g_u8UartWritePtr >= UART_RECV_MAX)
            {
              g_u8UartWritePtr = 0;
            }
        }
        
        osal_start_timerEx( SCAPP_TaskID, SCAPP_UART_PRASE_EVT, 5 );
                
  }
  
}

/* 串口发送函数 */
void sbpSerialAppWrite(uint8 *pBuffer, uint16 length)
{
  HalUARTWrite (SBP_UART_PORT, pBuffer, length);
}

/*
打印一个字符串
str不可以包含0x00，除非结尾
*/
void SerialPrintString(uint8 str[])
{
  HalUARTWrite (HAL_UART_PORT_0, str, osal_strlen((char*)str));
}
/*
打印指定的格式的数值
参数
title,前缀字符串
value,需要显示的数值
format,需要显示的进制，十进制为10,十六进制为16
*/
void SerialPrintValue(char *title, uint16 value, uint8 format)
{
  uint8 tmpLen;
  uint8 buf[256];
  uint32 err;

  tmpLen = (uint8)osal_strlen( (char*)title );
  osal_memcpy( buf, title, tmpLen );
  buf[tmpLen] = ' ';
  err = (uint32)(value);
  _ltoa( err, &buf[tmpLen+1], format );
  SerialPrintString(buf);		
}

/* BLE发送函数 */
void sbpSerialAppSendNoti(uint8 *pBuffer,uint16 length)
{
  uint8 len;
  static attHandleValueNoti_t pReport;
  
  if(length > 20)
    len = 20;
  else
    len = length;
  
  pReport.handle = 0x2E;
  pReport.len = len;
  
  pReport.pValue = GATT_bm_alloc(0, ATT_HANDLE_VALUE_NOTI,
                                  20, NULL);
  
  osal_memcpy(pReport.pValue, pBuffer, len);

  if ( GATT_Notification( 0, &pReport, FALSE ) != SUCCESS )
  {
    GATT_bm_free((gattMsg_t *)&pReport, ATT_HANDLE_VALUE_NOTI);
  }
}
/*********************************************************************
* @fn      Minus_PtrMv
*
* @brief   caculate D-value current point with Minnd
*
* @param   uint8 Current_Ptr,uint8 Minnd
*
* @return  void
*/
uint16 Minus_PtrMv(uint16 Current_Ptr,uint16 Minnd)
{
  uint16 Stp_Mv;
  if (Current_Ptr >= Minnd)
    Stp_Mv = Current_Ptr - Minnd;
  else
    Stp_Mv = Current_Ptr + UART_RECV_MAX - Minnd;
  return Stp_Mv;
}

/*********************************************************************
* @fn      Plus_PtrMv
*
* @brief   caculate SUM current point with Minnd
*
* @param   uint8 Current_Ptr,uint8 addnd
*
* @return  void
*/
uint16 Plus_PtrMv(uint16 Current_Ptr,uint16 addnd)
{
  uint16 Stp_Mv;
  Stp_Mv = (Current_Ptr + addnd) % UART_RECV_MAX;
  return Stp_Mv;
}
/* Uart解析函数 */
uint8 sbpUartDataParse()
{
  uint8 u8Len = 0;
  uint8 u8BuffLen = 0;
  static bool find_head = FALSE;
  static bool find_tail = FALSE;
  static uint16 cmd_head_index = 0;
  static uint16 cmd_tail_index = 0;
  
  
  while(g_u8UartReadPtr != g_u8UartWritePtr)
  {
    if(g_u8UartData[g_u8UartReadPtr] == 0xF5)
    {
      find_head = TRUE;
      cmd_head_index = g_u8UartReadPtr;
      cmd_tail_index = cmd_head_index;
      
      //缓冲区中数据长度
      u8BuffLen = Minus_PtrMv(g_u8UartWritePtr, g_u8UartReadPtr);
      if(u8BuffLen < 2)
      {
        return 0;
      }
      else
      {
        //协议数据长度
        u8Len = g_u8UartData[Plus_PtrMv(cmd_head_index, 1)];
        if(u8Len > 20)
        {
          //处理指针
          g_u8UartReadPtr = Plus_PtrMv(g_u8UartReadPtr, 2);
         
          //clear cmd flag
          find_head = FALSE;
          find_tail = FALSE;
          cmd_head_index = 0;
          cmd_tail_index = 0;
          return 0;
        }
        else if(u8Len <= (u8BuffLen - 2))
        {
          find_tail = TRUE;
            
          if(find_tail == TRUE)
          {
            for( uint16 i = 0, j = cmd_head_index; i < u8Len + 2; i++, j = Plus_PtrMv(j,1) )
            {
              g_Praseu8UartData[i] = g_u8UartData[j];
            }
            
            //处理指针
            g_u8UartReadPtr = Plus_PtrMv(g_u8UartReadPtr, u8Len + 2);
            //clear cmd flag
            find_head = FALSE;
            find_tail = FALSE;
            cmd_head_index = 0;
            cmd_tail_index = 0;
            
            return (u8Len + 2);
          }
        }
        else
        {
          return 0;
        }
      }

    }
    else
    {
      g_u8UartReadPtr = Plus_PtrMv(g_u8UartReadPtr, 1);
    }
  }
  
  return 0;
}
