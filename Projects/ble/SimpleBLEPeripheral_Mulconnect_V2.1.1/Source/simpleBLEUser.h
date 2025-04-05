#ifndef SIMPLEBLEUSER_H
#define SIMPLEBLEUSER_H

#ifdef __cplusplus
extern "C"
{
#endif
  
/*********************宏定义**********************************/   
#define T1STAT_CHOIF            (1<<0)  //定时器1的通道0状态位  
#define T1STAT_CH1IF            (1<<1)  //定时器1的通道1状态位  
#define T1STAT_CH2IF            (1<<2)  //定时器1的通道2状态位  
#define T1STAT_CH3IF            (1<<3)  //定时器1的通道3状态位  
#define T1STAT_CH4IF            (1<<4)  //定时器1的通道4状态位  

  
typedef enum _tagEmPwmMode
{
  PWM_SITUATION_ONE = 0,
  PWM_SITUATION_TWO,
  
}emPwmMode;
typedef struct _tagStPwmSetPara
{ 
  emPwmMode emPwmSituation;
  uint16 u16TimeBase_T0;
  uint16 u16TimeBase_T1;
  uint16 u16TimeBase_T2;
  uint16 u16TimeBase_T3;
  uint16 u16TimeBase_T4;
  uint16 u16TimeBase_T5;
  
}stPwmSetPara;

//配置Timer1/3/4的标定频率
void TimerClkConfig(void);

//低频(Timer1硬PWM暂时弃用)  
void PwmLowFreInit(void);
void PwmLowFreSetCycle(uint8 u8Cycle);

//高频:1K-30KHz
void PwmHighFreInit(void);
void PwmHighFreSetCycle(uint8 u8Cycle);

//低频:1-30Hz
void PwmLowFrequencyInit(void);   
void PwmLowFrequencySetCycle(uint8 u8Cycle, uint8 u8Pulse, uint16 u16Interva, \
                                  uint16 u16Start, emPwmMode emPwmSitu);


#ifdef __cplusplus
}
#endif

#endif