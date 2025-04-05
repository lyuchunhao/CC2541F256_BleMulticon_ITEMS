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
#include "simpleBLEUser.h"



uint16 g_u16TimeBaseCycleCnt = 0;      //循环计数 20us加1
stPwmSetPara LowFrePwmSetPara;



/* Timer1硬PWM: 1.标定频率为8M单次计数16us两通道 2. 覆盖1-30Hz */
/* Timer1硬PWM: 2. P1_0 及 P1_1                                */
/******************************************************************************/
/* 配置定时器标定频率为8M，默认为32M */
void TimerClkConfig(void)
{
#if 0                                  // 裸机程序配置32M晶振
  CLKCONCMD &= ~0x40;                  // 设置系统时钟源为 32MHZ晶振
  while(CLKCONSTA & 0x40);             // 等待晶振稳定 
  CLKCONCMD &= ~0x47;                  // 设置系统主时钟频率为 32MHZ
#endif
  
  //使用定时器PWM想要实现高频1K-30K,低频1-30Hz，则设置为8M最好
  CLKCONCMD &= ~(BV(3)|BV(4)|BV(5));   //定时器标记输出设置为32M
  CLKCONCMD |= BV(4);                  //定时器标记输出设置为8M
  while((CLKCONSTA & 0x10) != 0x10);
}

//Timer1 通道0/1  P1_0 P1_1 (需要配置定时器标定频率 = 8M, 模模式)
void PwmLowFreInit(void)
{
  //设置pwm端口为输出
  P1DIR|= BV(0)|BV(1);
  //设置pwm端口为外设端口，非gpio
  P1SEL|= BV(0)|BV(1);
  //通道12分别为P1.1P1.0
  PERCFG |= 0x40;             // Move Timer1 to alternate2 location

  // Initialize Timer 1
  T1CTL = 0x0C;               // Div = 128, CLR, MODE = Suspended  128/8 = 16us        
  //T1CCTL1 = 0x0C;           // IM = 0; CMP = Clear output on compare; Mode = Compare
  //T1CCTL2 = 0x04;           // IM = 0; CMP = Set   output on compare; Mode = Compare
  //T1CCTL3 = 0x0C;           // IM = 0, CMP = Clear output on compare; Mode = Compare

  T1CCTL1 = 0x2C;
  T1CCTL2 = 0x34;
  
  T1CNTL = 0;                 // Reset timer to 0;

  T1CCTL0 = 0x3C;             // IM = 1; CMP = Clear output on compare; Mode = Compare    
  T1CC0H = 0x0C;              //0x0C35 = 3125    T = 3125*16us = 50ms(20Hz)
  T1CC0L = 0x35;            
          
  T1CC1H = 0x06;              //0x061A = 1562       
  T1CC1L = 0x1A;
  T1CC2H = 0x06;              
  T1CC2L = 0x1A;

  //EA=1;
  //IEN1 |= 0x02;              // Enable T1 cpu interrupt
}
/* 更新Pwm的值:1-30Hz 步进1K */
void PwmLowFreSetCycle(uint8 u8Cycle)
{
  uint32 u32TimeCnt = 0;
  
  if(u8Cycle < 1 || u8Cycle >30)
  {
    SerialPrintString("PwmHighFreSetCycle para error, please input 1-30 \n");
    return;
  }

  // Set up the timer registers
  u32TimeCnt = 62500/u8Cycle;
  //u32TimeCnt = 1000;
  T1CC0L = (uint8)u32TimeCnt;
  T1CC0H = (uint8)(u32TimeCnt >> 8);
  //SerialPrintValue("Cycl", u32TimeCnt, 10);

  //50%占空比
  u32TimeCnt = u32TimeCnt/2;
  //u32TimeCnt = 500;
  T1CC1L = (uint8)u32TimeCnt;
  T1CC1H = (uint8)(u32TimeCnt >> 8);
  T1CC2L = (uint8)u32TimeCnt;
  T1CC2H = (uint8)(u32TimeCnt >> 8);

  // Reset timer
  T1CNTL = 0;
  
  // Start timer in modulo mode.
  T1CTL |= 0x02; 

}
/******************************************************************************/


/* Timer3/4硬PWM: 1. 高频1K-30K分别选择Timer3的0通道和Timer4的0通道 */
/* Timer3/4硬PWM: 2. P1_3及P2_0  [标定主频:32M]                      */
/******************************************************************************/
//Timer3 通道0  P1_3 (定时器标定频率 = 32M, 正计数/倒计数模式)
void InitTimer3()
{
    //PERCFG |= 0x20;           // Timer 3 Alternate location 2    
    P1DIR |= 0x08;              // P1_3 = output    
    P1SEL |= 0x08;              // Peripheral function on P1_3  
    
    T3CTL &= ~0x10;             // Stop timer 3 (if it was running)    
    T3CTL |= 0x04;              // Clear timer 3    
    T3CTL &= ~0x08;             // Disable Timer 3 overflow interrupts    
    T3CTL |= 0x03;              // Timer 3 mode = 3 - Up/Down    
    T3CTL |= 0xC0;              // 64分频   64/32M = 2us
    
    T3CCTL0 &= ~0x40;           // Disable channel 0 interrupts    
    T3CCTL0 |= 0x04;            // Compare mode    
    T3CCTL0 |= 0x18;            // 011  在比较正计数时设置输出，在0清除
    
    T3CC0 = 10;                  //上限值设置为10   
    
    //T3CTL |= 0x10;             // start timer 3    
  

}
//Timer4 通道0  P2_0   (定时器标定频率 = 32M，正计数/倒计数模式)
void InitTimer4()
{

    PERCFG |= 0x10;             // Timer 4 Alternate location 2    
    P2DIR |= 0x01;              // P2_0 = output    
    P2SEL |= 0x01;              // Peripheral function on P2_0   
    
    T4CTL &= ~0x10;             // Stop timer 4 (if it was running)    
    T4CTL |= 0x04;              // Clear timer 4    
    T4CTL &= ~0x08;             // Disable Timer 4 overflow interrupts    
    T4CTL |= 0x03;              // Timer 4 mode = 4 - Up/Down    
    T4CTL |= 0xC0;              // 16分频    64/32M = 2us
    
    T4CCTL0 &= ~0x40;           // Disable channel 0 interrupts    
    T4CCTL0 |= 0x04;            // Compare mode    
    T4CCTL0 |= 0x20;            // 100  在比较正计数时设置清除，在0输出      
    
    T4CC0 = 10;                 //上限值设置为10   
    
    //T4CTL |= 0x10;            // start timer 3    

}
//停止高频PWM输出
void PwmHighFreStop()
{
    T3CTL &= ~0x10;
    T4CTL &= ~0x10;
}
//开始高频PWM输出
void PwmHighFreStart()
{
    T3CTL |= 0x10;
    T4CTL |= 0x10;
    
    T3CTL |= 0x04; 
    T4CTL |= 0x04; 
}
//高频PWM初始化
void PwmHighFreInit(void)
{
  InitTimer3();
  InitTimer4();
}
//u8Cycle:1K-30K
void PwmHighFreSetCycle(uint8 u8Cycle)
{
  uint8 u8TimeValue = 0;
  
  if(u8Cycle < 1 || u8Cycle >30)
  {
    SerialPrintString("PwmHighFreSetCycle para error, please input 1-30 \n");
    return; 
  }
  
  //stop Timer
  PwmHighFreStop();
  switch(u8Cycle)
  {
    case 1:
    case 2:
           //8bit定时器，需要64分频 64/32M = 2us  
           T3CTL &= ~0xE0;
           T4CTL &= ~0xE0;
           T3CTL |= 0xC0; 
           T4CTL |= 0xC0;
           u8TimeValue = (uint8)(250/u8Cycle);
           T3CC0 = u8TimeValue;
           T4CC0 = u8TimeValue;
           break;
           
    case 3:
    case 4:
    case 5:
           //8bit定时器，需要32分频 32/32M = 1us  
           T3CTL &= ~0xE0;
           T4CTL &= ~0xE0;
           T3CTL |= 0xA0; 
           T4CTL |= 0xA0; 
           u8TimeValue = (uint8)(500/u8Cycle);
           T3CC0 = u8TimeValue;
           T4CC0 = u8TimeValue;
           break;
           
    case 6:
    case 7:
    case 8:
           //8bit定时器，需要16分频 16/32M = 1/2us  
           T3CTL &= ~0xE0;
           T4CTL &= ~0xE0;
           T3CTL |= 0x80; 
           T4CTL |= 0x80; 
           u8TimeValue = (uint8)(1000/u8Cycle);
           T3CC0 = u8TimeValue;
           T4CC0 = u8TimeValue;
           break;
           
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
    case 16:
           //8bit定时器，需要8分频 8/32M = 1/4us  
           T3CTL &= ~0xE0;
           T4CTL &= ~0xE0;
           T3CTL |= 0x60; 
           T4CTL |= 0x60;   
           u8TimeValue = (uint8)(2000/u8Cycle);
           T3CC0 = u8TimeValue;
           T4CC0 = u8TimeValue;
           break;
           
    case 17:
    case 18:
    case 19:
    case 20:
    case 21:
    case 22:
    case 23:
    case 24:
    case 25:
    case 26:
    case 27:
    case 28:
    case 29:
    case 30:
           //8bit定时器，需要4分频 4/32M = 1/8us  
           T3CTL &= ~0xE0;
           T4CTL &= ~0xE0;
           T3CTL |= 0x40; 
           T4CTL |= 0x40;  
           u8TimeValue = (uint8)(4000/u8Cycle);
           T3CC0 = u8TimeValue;
           T4CC0 = u8TimeValue;
           break;
                    
    default :
           break;
      
  }
  
  //Start Timer
  PwmHighFreStart();
}
/******************************************************************************/



/* Timer1定时中断: 1. 低频1-30Hz 脉宽0.1-1ms IO翻转实现                       */
/* Timer1定时中断: 2. P1_0及P1_1  [标定主频:32M]                              */
/* Timer1定时中断: 3. 定时最小刻度为20us                                      */
/******************************************************************************/
/* 定时器1初始化 (定时器标定频率 = 32M) */
void InitTimer1(void)      
{  
  //定时器1配置  
  T1CTL = (0<<2)|(2<<0);                //0000(reserved)、00(1分频，1/32M=1/32Us、10（Modulo）  
  T1CNTL = 0;                           //清除计数器  
    
  //定时器1的通道0配置  
  T1CCTL0 = (1<<6)|(7<<3)|(1<<2)|(0<<0);//Enables interrupt request、Initialize output pin. CMP[2:0] 
                                        //is not changed、Compare mode、 No capture  

  T1CC0L = 640%256;                     //低位   定时20us
  T1CC0H = 640/256;                     //高位  
   
  //中断配置  
  IEN1 |= (1<<1);                       //定时器1中断使能  
  EA = 1;
   
} 

/******************************************************************************
*函 数 名：PwmLowFrequencySetCycle
*功    能：低频PWM参数设置函数
*入口参数：u8Cycle：1-30 Hz
*入口参数：脉宽 1-10 [0.1ms - 1ms]
*入口参数：u16Interval: 间隔 (脉宽间隔最小为20us) 
*入口参数：u16Start:   脉宽起始 1LSB = 20us
*入口参数：u16PwmMode: 0 - 情形1 1 - 情形2
*出口参数：情形1                                   情形2
          |     ____                     |         |              ____         |
          |____|    |____________________| Pwm A   |_____________|    |________|
          |T0  T1   T2   ____            |         |       ____ T3    T4     T5|
          |_____________|    |___________| Pwm B   |______|    |_______________|
          |             T3   T4        T5|         |T0    T1   T2              |

******************************************************************************/ 
void PwmLowFrequencySetCycle(uint8 u8Cycle, uint8 u8Pulse, uint16 u16Interva, \
                                  uint16 u16Start, emPwmMode emPwmSitu)
{
  uint16 u16T0,u16T1,u16T2,u16T3,u16T4,u16T5;
  
  if(u8Cycle < 1 || u8Cycle > 30)
  {
    return; 
  }
  if(u8Pulse < 1 || u8Pulse > 10)
  {
    return; 
  }
  if(u16Interva < 1 || u16Interva > 50000)
  {
    return; 
  }
  
  u16T5 = 50000/u8Cycle;             //周期计数 
  u16T0 = 0;                         //周期开始  0点
  u16T1 = u16T0 + u16Start;          //脉宽起始  前言
  u16T2 = u16T1 + u8Pulse*100/20;    //脉宽宽度  T2-T1
  u16T3 = u16T2 + u16Interva;        //脉宽间隔  T3-T2
  u16T4 = u16T3 + u8Pulse*100/20;    //第二通道  脉宽
  
  if(u16T4 > u16T5)
  {
    return; 
  }
  else
  {
    LowFrePwmSetPara.emPwmSituation  = emPwmSitu;
    LowFrePwmSetPara.u16TimeBase_T0 = u16T0;
    LowFrePwmSetPara.u16TimeBase_T1 = u16T1;
    LowFrePwmSetPara.u16TimeBase_T2 = u16T2;
    LowFrePwmSetPara.u16TimeBase_T3 = u16T3;
    LowFrePwmSetPara.u16TimeBase_T4 = u16T4;
    LowFrePwmSetPara.u16TimeBase_T5 = u16T5;  
  }
  
}
//低频1-30Hz初始化
void PwmLowFrequencyInit(void)
{
 
  P1DIR |= BV(0)| BV(1) ;               //定义为输出
  P1SEL &= ~(BV(0)|BV(1));              //定义一般GPIO
  P1_0 = 0;
  P1_1 = 0;
  
  //10Hz 脉宽1ms 间隔10*20us = 200us 起始10*20=200us
  PwmLowFrequencySetCycle(10, 10, 10, 10, PWM_SITUATION_TWO);
  InitTimer1();
}
//中断调用电平翻转函数(实验证明：如果中断中调用实际频率会差好多)
void  PwmLowFrequencyIOFlip(void)
{
    if(g_u16TimeBaseCycleCnt == LowFrePwmSetPara.u16TimeBase_T0)
    {
        P1_0 = 0;
        P1_1 = 0;
    }
    else if(g_u16TimeBaseCycleCnt == LowFrePwmSetPara.u16TimeBase_T1)
    {
        if(LowFrePwmSetPara.emPwmSituation == PWM_SITUATION_ONE)
        {
          P1_0 = 1;
          P1_1 = 0;
        }
        else
        {
          P1_0 = 0;
          P1_1 = 1;
        }

    }
    else if(g_u16TimeBaseCycleCnt== LowFrePwmSetPara.u16TimeBase_T2)
    {
        P1_0 = 0;
        P1_1 = 0;
    }
    else if(g_u16TimeBaseCycleCnt == LowFrePwmSetPara.u16TimeBase_T3)
    {      
        if(LowFrePwmSetPara.emPwmSituation == PWM_SITUATION_ONE)
        {
          P1_0 = 0;
          P1_1 = 1;
        }
        else
        {
          P1_0 = 1;
          P1_1 = 0;
        }
    }
    else if(g_u16TimeBaseCycleCnt == LowFrePwmSetPara.u16TimeBase_T4)
    {
        P1_0 = 0;
        P1_1 = 0;
    }
    else if(g_u16TimeBaseCycleCnt >= LowFrePwmSetPara.u16TimeBase_T5)
    {
        P1_0 = 0;
        P1_1 = 0;
        
        g_u16TimeBaseCycleCnt = 0;
    }
}

/******************************************************************************
*函 数 名：Timer1_ISR
*功    能：定时器3中断服务程序
*入口参数：每20us进入该中断一次
*出口参数：无
******************************************************************************/ 
#pragma vector = T1_VECTOR     
__interrupt void Timer1_ISR(void)     
{             
  unsigned char flags = T1STAT;  
    
  //通道0  
  if(flags & T1STAT_CHOIF)  
  {  
#if 0   
    //如果调用函数频率会差很多
    PwmLowFrequencyIOFlip();
    
#else
    if(g_u16TimeBaseCycleCnt == LowFrePwmSetPara.u16TimeBase_T0)
    {
        P1_0 = 0;
        P1_1 = 0;
    }
    else if(g_u16TimeBaseCycleCnt == LowFrePwmSetPara.u16TimeBase_T1)
    {
        if(LowFrePwmSetPara.emPwmSituation == PWM_SITUATION_ONE)
        {
          P1_0 = 1;
          P1_1 = 0;
        }
        else
        {
          P1_0 = 0;
          P1_1 = 1;
        }

    }
    else if(g_u16TimeBaseCycleCnt== LowFrePwmSetPara.u16TimeBase_T2)
    {
        P1_0 = 0;
        P1_1 = 0;
    }
    else if(g_u16TimeBaseCycleCnt == LowFrePwmSetPara.u16TimeBase_T3)
    {      
        if(LowFrePwmSetPara.emPwmSituation == PWM_SITUATION_ONE)
        {
          P1_0 = 0;
          P1_1 = 1;
        }
        else
        {
          P1_0 = 1;
          P1_1 = 0;
        }
    }
    else if(g_u16TimeBaseCycleCnt == LowFrePwmSetPara.u16TimeBase_T4)
    {
        P1_0 = 0;
        P1_1 = 0;
    }
    else if(g_u16TimeBaseCycleCnt >= LowFrePwmSetPara.u16TimeBase_T5)
    {
        P1_0 = 0;
        P1_1 = 0;
        
        g_u16TimeBaseCycleCnt = 0;
    }
#endif  
    
    g_u16TimeBaseCycleCnt++;
 
  }
    
} 

/******************************************************************************/




/******************************************************************************
*函 数 名：Timer3_ISR
*功    能：定时器3中断服务程序
*入口参数：无
*出口参数：暂时尚未使用
******************************************************************************/
#pragma vector = T3_VECTOR 
__interrupt void UserTimer3_ISR(void) 
{ 
  //不软件清除中断位也能正常运行,但为了严谨还是加上吧
   IRCON &= ~0x08;
   TIMIF &= ~0x11;  
}




