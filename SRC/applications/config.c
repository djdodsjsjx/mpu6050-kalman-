#include "config.h"

float Velocity_KP=10,Velocity_KI=8;	              //速度控制PID参数
float	Position_KP=10,Position_KI=0.38,Position_KD=50;  //位置控制PID参数
long int Motor_A,Motor_B,Motor_C,Motor_D;           //电机PWM变量
long int Position_A,Position_B,Position_C,Position_D,Rate_A,Rate_B,Rate_C,Rate_D; //PID控制相关变量
long int Target_A,Target_B,Target_C,Target_D;       //电机目标值
int Encoder_A,Encoder_B,Encoder_C,Encoder_D;        //编码器的脉冲计数
	
void data_config()
{
	
	
}

void init_config(void)
{  
	SysTick_Configuration();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	Scheduler_Setup();
//	Uart2_Init(1000000);
//	pwm_servo_init();	
//	Usart3_Init(500000);	
//	Usart3_Init(9600);	
//	OLED_Init();
	MPU6050_initialize();
	TIM_INIT();
	uart_init(500000); 
}

