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
//introduce:    矩阵键盘IO初始化      
//author:       lyuchunhao        
//changetime:   2018-05-18      
//describe:     P1_0 - P1_3 定义为输入：B1 - B4   
//describe:     P0_4 - P0_7,P0_1 定义为输出 A1 - A5
//******************************************************************************   
void UserKeyboard(void)
{
  //定义B1-B4为输入
  P1DIR &= ~(BV(0)|BV(1)|BV(2)|BV(3)) ;          //定义为输入
  P1SEL &= ~(BV(0)|BV(1)|BV(2)|BV(3));           //定义一般GPIO
  //P1INP &= ~(BV(4)|BV(5)|BV(6)|BV(7)|BV(1));   //定义上拉下拉模式
  //P2INP &= ~BV(6);                             //对端口1所有端口设置上拉
  
  //定义A1-A5为输出
  P0DIR |= BV(4)|BV(5)|BV(6)|BV(7)|BV(1) ;       //定义为输入
  P0SEL &= ~(BV(4)|BV(5)|BV(6)|BV(7)|BV(1));     //定义一般GPIO
  
}



//******************************************************************************          
//name:         UserTimer1Init()          
//introduce:    定时器1的初始化        
//parameter:    none         
//return:       none        
//author:       lyuchunhao        
//changetime:   2018-05-18    
//describe:     32M进行128分频 - >F = 250K t = 4us
//describe:     如果定时1ms(1000Hz)，计数 = 1ms/4us = 250          
//******************************************************************************     
void UserTimer1Init(void)      
{  
  //定时器1配置  
  T1CTL = (3<<2)|(2<<0);                         //0000(reserved)、11(128分频，32M/128=250K、10 Modulo) 
  T1CNTL = 0;                                    //清除计数器  
    
  //定时器1的通道0配置  
  T1CCTL0 = (1<<6)|(7<<3)|(1<<2)|(0<<0);         //Enables interrupt request、Initialize output   
  T1CC0H = 250/256;                              //pin. CMP[2:0] is not changed、Compare mode、 No capture
  T1CC0L = 250%256;                              //低位   
   
  //中断配置  
  IEN1 |= (1<<1);                                //定时器1中断使能  
} 

//******************************************************************************    
//name:         UserTimer1_ISR          
//introduce:    定时器1的中断服务函数        
//parameter:    none         
//return:       none        
//author:       lyuchunhao        
//changetime:   2018-05-18 
//describe:     1ms中断定时函数
//******************************************************************************    
#pragma vector = T1_VECTOR     
__interrupt void UserTimer1_ISR(void)     
{             
  unsigned char flags = T1STAT;  
    
  //通道0  
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
    
    //清除定时器1通道0中断(似乎不清除也能正常运行)
    T1STAT &= ~BV(0);
  } 
  
} 


