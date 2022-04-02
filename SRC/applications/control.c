#include "control.h"

void TIM4_IRQHandler(void)
{

	if(TIM_GetITStatus(TIM4,TIM_IT_Update))
	{
//				Encoder_A=Read_Encoder(2);
//				PWMA=Postion_PID(Encoder_A,Target_A);
//				DT_SendF1(Encoder_A,Target_A,0,0,0,0);
   }	
	
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update|TIM_IT_CC1);
}

int Postion_PID (int Encoder,int Target)
{ 	
		static int error,error_sum,last_error,Pwm;  
		error=Encoder-Target;
	  if(error<0)  MOTORA1=1, MOTORA2=0;
	  if(error>0)  MOTORA1=0, MOTORA2=1;
		error_sum+=error; 
		if(error_sum>1000)error_sum=1000; 
		if(error_sum<-1000)error_sum=-1000;
		Pwm=error*Position_KP+error_sum*Position_KI+(error-last_error)*Position_KD;
		last_error=error;
	  if(Pwm>6900)   Pwm=6900;
		if(Pwm<-6900)  Pwm=-6900;	
		return ABS(Pwm);
}

/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ�����PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d)
{
	    int siqu=0;
    	if(motor_a>0)			MOTORA1=1, MOTORA2=0;
			else  	          MOTORA1=0,MOTORA2=1;
	    PWMA=ABS(motor_a)+siqu;
	
}

/**************************************************************************
�������ܣ�����PWM��ֵ 
��ڲ�������ֵ
����  ֵ����
**************************************************************************/
void Xianfu_Pwm(int amplitude)
{	
    if(Motor_A<-amplitude) Motor_A=-amplitude;	
		if(Motor_A>amplitude)  Motor_A=amplitude;	

}
/**************************************************************************
�������ܣ�λ��PID���ƹ������ٶȵ�����
��ڲ������ޡ���ֵ
����  ֵ����
**************************************************************************/
void Xianfu_Velocity(int amplitude_A,int amplitude_B,int amplitude_C,int amplitude_D)
{	
    if(Motor_A<-amplitude_A) Motor_A=-amplitude_A;	//λ�ÿ���ģʽ�У�A����������ٶ�
		if(Motor_A>amplitude_A)  Motor_A=amplitude_A;	  //λ�ÿ���ģʽ�У�A����������ٶ�
	  if(Motor_B<-amplitude_B) Motor_B=-amplitude_B;	//λ�ÿ���ģʽ�У�B����������ٶ�
		if(Motor_B>amplitude_B)  Motor_B=amplitude_B;		//λ�ÿ���ģʽ�У�B����������ٶ�
	  if(Motor_C<-amplitude_C) Motor_C=-amplitude_C;	//λ�ÿ���ģʽ�У�C����������ٶ�
		if(Motor_C>amplitude_C)  Motor_C=amplitude_C;		//λ�ÿ���ģʽ�У�C����������ٶ�
	  if(Motor_D<-amplitude_D) Motor_D=-amplitude_D;	//λ�ÿ���ģʽ�У�C����������ٶ�
		if(Motor_D>amplitude_D)  Motor_D=amplitude_D;		//λ�ÿ���ģʽ�У�C����������ٶ�
}


/**************************************************************************
�������ܣ�����PI������
��ڲ���������������ֵ��Ŀ���ٶ�
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ 
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  �Դ����� 
pwm�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //����ʽPI������
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   //������һ��ƫ�� 
	 return Pwm;                         //�������
}
int Incremental_PI_B (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //����ʽPI������
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   //������һ��ƫ�� 
	 return Pwm;                         //�������
}
int Incremental_PI_C (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                                  //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //����ʽPI������
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   //������һ��ƫ�� 
	 return Pwm;                         //�������
}
int Incremental_PI_D (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                                  //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //����ʽPI������
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   //������һ��ƫ�� 
	 return Pwm;                         //�������
}





void lanya(u8 data)
{
	switch(data)
	{
		case 1:  break;
		case 2:  break;
		case 3:  break;
		case 4:  break;
		case 5:  break;
		case 6:  break;
		case 7:  break;
		case 8:  break;
		case 9:  break;
		case 10: break;
		case 11: break;
		case 12:  break;
		default:       ;
	}
}

