#ifndef __CONFIG_H
#define __CONFIG_H

#include "stm32f10x.h"
#include "sys.h"
#include "usart.h"
#include "usart3.h"
#include "pwm.h"
#include "servo.h"
#include "uart2.h"
#include "openmv.h"
#include "calculate.h"
#include "control.h"
#include "math.h"
#include "stdlib.h"
#include "SCSCL.h"
#include "INST.h"
#include "SCS.h"
#include "uart2.h"
#include "DT.h"
#include "Scheduler.h"
#include "time.h"
#include "filter.h"
#include "hcsr04.h"
#include "mpu6050.h"
#include "mpu9250.h"
#include "mpuiic.h"
#include "oled.h"
#include "show.h"
#include "oled_spi.h"
#include "my_math.h"
#include "led.h"
#include "encoder.h"
#include "exti.h"
#include "motor.h"



#define NVIC_TIME_P       1					  
#define NVIC_TIME_S       3

#define NVIC_USART1_P       3					
#define NVIC_USART1_S       3

#define NVIC_USART2_P       1				
#define NVIC_USART2_S       0

#define NVIC_USART3_P       1					
#define NVIC_USART3_S       2

extern float Velocity_KP,Velocity_KI;	              //速度控制PID参数
extern float	Position_KP,Position_KI,Position_KD;  //位置控制PID参数
extern long int Motor_A,Motor_B,Motor_C,Motor_D;           //电机PWM变量
extern long int Position_A,Position_B,Position_C,Position_D,Rate_A,Rate_B,Rate_C,Rate_D; //PID控制相关变量
extern long int Target_A,Target_B,Target_C,Target_D;       //电机目标值
extern int Encoder_A,Encoder_B,Encoder_C,Encoder_D;        //编码器的脉冲计数

void data_config(void);
void init_config(void);
#endif

