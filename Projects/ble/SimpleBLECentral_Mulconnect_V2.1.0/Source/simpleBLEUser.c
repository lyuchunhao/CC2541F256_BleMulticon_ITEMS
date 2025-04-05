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


#include "simpleBLEUser.h"
#include "simpleBLECentral.h"



uint32 g_u32TimeBase = 0;



//******************************************************************************          
//name:         UserKeyboard()          
//introduce:    �������IO��ʼ��      
//author:       lyuchunhao        
//changetime:   2018-05-18      
//describe:     P1_0 - P1_3 ����Ϊ���룺B1 - B4   
//describe:     P0_4 - P0_7,P0_1 ����Ϊ��� A1 - A5
//******************************************************************************   
void UserKeyboard(void)
{
  //����B1-B4Ϊ����
  P1DIR &= ~(BV(0)|BV(1)|BV(2)|BV(3)) ;          //����Ϊ����
  P1SEL &= ~(BV(0)|BV(1)|BV(2)|BV(3));           //����һ��GPIO
  //P1INP &= ~(BV(4)|BV(5)|BV(6)|BV(7)|BV(1));   //������������ģʽ
  //P2INP &= ~BV(6);                             //�Զ˿�1���ж˿���������
  
  //����A1-A5Ϊ���
  P0DIR |= BV(4)|BV(5)|BV(6)|BV(7)|BV(1) ;       //����Ϊ����
  P0SEL &= ~(BV(4)|BV(5)|BV(6)|BV(7)|BV(1));     //����һ��GPIO
  
}



//******************************************************************************          
//name:         UserTimer1Init()          
//introduce:    ��ʱ��1�ĳ�ʼ��        
//parameter:    none         
//return:       none        
//author:       lyuchunhao        
//changetime:   2018-05-18    
//describe:     32M����128��Ƶ - >F = 250K t = 4us
//describe:     �����ʱ1ms(1000Hz)������ = 1ms/4us = 250          
//******************************************************************************     
void UserTimer1Init(void)      
{  
  //��ʱ��1����  
  T1CTL = (3<<2)|(2<<0);                         //0000(reserved)��11(128��Ƶ��32M/128=250K��10 Modulo) 
  T1CNTL = 0;                                    //���������  
    
  //��ʱ��1��ͨ��0����  
  T1CCTL0 = (1<<6)|(7<<3)|(1<<2)|(0<<0);         //Enables interrupt request��Initialize output   
  T1CC0H = 250/256;                              //pin. CMP[2:0] is not changed��Compare mode�� No capture
  T1CC0L = 250%256;                              //��λ   
   
  //�ж�����  
  IEN1 |= (1<<1);                                //��ʱ��1�ж�ʹ��  
} 

//******************************************************************************    
//name:         UserTimer1_ISR          
//introduce:    ��ʱ��1���жϷ�����        
//parameter:    none         
//return:       none        
//author:       lyuchunhao        
//changetime:   2018-05-18 
//describe:     1ms�ж϶�ʱ����
//******************************************************************************    
#pragma vector = T1_VECTOR     
__interrupt void UserTimer1_ISR(void)     
{             
  unsigned char flags = T1STAT;  
    
  //ͨ��0  
  if(flags & T1STAT_CHOIF)  
  {  
    //do something
    g_u32TimeBase++;
    if(g_u32TimeBase >= 50000)
    {
      g_u32TimeBase = 0;
    }
    
    if(g_u32TimeBase % 1000 == 0)
    {
      SerialPrintValue("T1: ", g_u32TimeBase, 10);     
      SerialPrintString("\n");
    }
    
    //�����ʱ��1ͨ��0�ж�(�ƺ������Ҳ����������)
    T1STAT &= ~BV(0);
  } 
  
} 


