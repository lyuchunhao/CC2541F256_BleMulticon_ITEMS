#ifndef _SERIAL_APP_H_
#define _SERIAL_APP_H_

#ifdef __cplusplus
extern "C"
{
#endif
  

#define SBP_UART_PORT                  HAL_UART_PORT_0
//#define SBP_UART_FC                  TRUE
#define SBP_UART_FC                    FALSE
#define SBP_UART_FC_THRESHOLD          48
#define SBP_UART_RX_BUF_SIZE           128
#define SBP_UART_TX_BUF_SIZE           128
#define SBP_UART_IDLE_TIMEOUT          6
#define SBP_UART_INT_ENABLE            TRUE
#define SBP_UART_BR                    HAL_UART_BR_57600 
  
  
#define UART_RECV_MAX                  128


// Serial Port Related
extern void SerialApp_Init(uint8 taskID);
extern void sbpSerialAppCallback(uint8 port, uint8 event);
void serialAppInitTransport();
void SerialPrintString(uint8 str[]);
void sbpSerialAppWrite(uint8 *pBuffer, uint16 length);
void sbpSerialAppSendNoti(uint8 *pBuffer,uint16 length);
void SerialPrintValue(char *title, uint16 value, uint8 format);

uint8 sbpUartDataParse();

#ifdef __cplusplus
}
#endif

#endif
