#ifndef SIMPLEBLEUSER_H
#define SIMPLEBLEUSER_H

#ifdef __cplusplus
extern "C"
{
#endif
  
/*********************�궨��**********************************/   
#define T1STAT_CHOIF            (1<<0)  //��ʱ��1��ͨ��0״̬λ  
#define T1STAT_CH1IF            (1<<1)  //��ʱ��1��ͨ��1״̬λ  
#define T1STAT_CH2IF            (1<<2)  //��ʱ��1��ͨ��2״̬λ  
#define T1STAT_CH3IF            (1<<3)  //��ʱ��1��ͨ��3״̬λ  
#define T1STAT_CH4IF            (1<<4)  //��ʱ��1��ͨ��4״̬λ  

#define KEY_B1_IN = P1_0
#define KEY_B2_IN = P1_1
#define KEY_B3_IN = P1_2
#define KEY_B4_IN = P1_3
  
#define KEY_A1_OUT = P0_1
#define KEY_A2_OUT = P0_4
#define KEY_A3_OUT = P0_5
#define KEY_A4_OUT = P0_6
#define KEY_A5_OUT = P0_7


extern void UserKeyboard(void);
extern void UserTimer1Init(void);  

#ifdef __cplusplus
}
#endif

#endif