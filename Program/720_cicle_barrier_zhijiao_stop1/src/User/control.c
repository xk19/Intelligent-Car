#include "include.h" 


//环岛标志位
unsigned char Turn_Left_Flag = 0;
unsigned char Turn_Right_Flag = 0;
unsigned char Turn_Flag = 0;
unsigned char Go_Out_Circle = 0;
unsigned char outsteer_flag = 0;
//
unsigned char RoadType = 0;
unsigned char Dir_last = 0;
extern unsigned char backcar;
extern unsigned char stopcar;
extern unsigned char stopmotor;
extern unsigned char rightflag;
extern unsigned char leftflag;
extern unsigned char turnflag;
//超声波测距
extern volatile uint32_t hc_time;
extern int distance;
extern unsigned char barrier;
extern unsigned char barrier_delay;
extern unsigned char ba_delay1S;
extern unsigned char ba_timer; 
extern unsigned char barrier_stage1; 
extern unsigned char barrier_stage2; 
extern unsigned char barrier_stage3; 

//声明电机速度输出值
extern volatile int outspeed;
extern volatile int outsteer;
//电感采集数值
extern int AD1;
extern int AD2;
extern int AD3;
extern int AD4;
extern int AD5;
extern int AD6;
extern int ADC1VAL;
extern int ADC2VAL;
extern int ADC3VAL;
extern int ADC4VAL;
extern int ADC5VAL;
extern int ADC6VAL;
unsigned int ADC1filter[10];
unsigned int ADC2filter[10];
unsigned int ADC3filter[10];
unsigned int ADC4filter[10];
unsigned int ADC5filter[10];
unsigned int ADC6filter[10];
//电感斜率数值
float SLOPE1 = 0;
float SLOPE2 = 0;
float SLOPE3 = 0;
float SLOPE4 = 0;
float SLOPE5 = 0;
float SLOPE6 = 0;
float FSLOPE1[3];
float FSLOPE2[3];
float FSLOPE3[3];
float FSLOPE4[3];
float FSLOPE5[3];
float FSLOPE6[3];
//速度采集数值
extern int FTMSPEED;
extern float nowspeed;
//方向环存储用
extern  float dir_error;
extern float dir_old_error;
extern float DuoP;
extern float DuoD;
//速度环数据
extern float speed_set;
extern float speed_error;
extern float speed_error_last;
extern float speed_error_last_last;
extern float speed_Kp;
extern float speed_Ki;
extern float speed_Kd;
//电机限幅输出函数
void motor_out()
{
  if(stopmotor==1)
  {
    outspeed = 0;
  }
	//限幅
	if(outspeed>=MOTOR_MAX)outspeed=MOTOR_MAX;
	if(outspeed<=-MOTOR_MAX)outspeed=-MOTOR_MAX;
	//判断方向并且输出
	if(outspeed>0)
	{
		FTM_PWM_Duty(FTM0,FTM_CH0,0);
		FTM_PWM_Duty(FTM0,FTM_CH1,outspeed);
	}
	else if(outspeed<0)
	{
		FTM_PWM_Duty(FTM0,FTM_CH0,-outspeed);
		FTM_PWM_Duty(FTM0,FTM_CH1,0);
	}
	else
	{
		FTM_PWM_Duty(FTM0,FTM_CH0,0);
		FTM_PWM_Duty(FTM0,FTM_CH1,0);
	}
}
//计算电机输出
void motor_cal()
{
  //判断停车标志位
  
  //如果设定速度比当前速度小，就增增加响应特性

  if(speed_set>=0)
  {  
    if((speed_set)<=(nowspeed-0.3))//下面的几个关系有子集关系，所以先判断最小的区域
    {
      //设定更小一些，>0.3
      speed_Kp = 90;
      speed_Kd = 1.5;
    }
    else if((speed_set)<=(nowspeed-0.15))
    {
      //设定更小一些，>0.1
      speed_Kp = 70;
      speed_Kd = 1.5;
    }
    else if((speed_set)<=(nowspeed))
    {
      //设定更小一些，>0
      speed_Kp = 30;
      speed_Kd = 1.5;
    }
    else if((speed_set-0.2)>(nowspeed))
    {
      //设定更大一些，>0.3
      speed_Kp = 50;
      speed_Kd = 1.5;
    }
    else if((speed_set)>(nowspeed))
    {
      //设定更大一些，>0.1
      speed_Kp = 30;
      speed_Kd = 1.5;
    }
  }
  else
  {
    speed_Kp = 30;
    speed_Kd = 1.5;
  }
  
  if(stopcar!=1)
  {
    speed_error = speed_set - nowspeed;//计算误差
    outspeed += (speed_error)*speed_Kp + /*speed_error*speed_Ki +*/ (speed_error-2*speed_error_last+speed_error_last_last)*speed_Kd;
    speed_error_last_last = speed_error_last;
    speed_error_last = speed_error;
    if(backcar)
    {
      speed_set = -0.7;
    }
  }
  //停车的话直接不计算了
  else
  {
    if(nowspeed>0.5)
    {
      outspeed = -700;
    }
    else
    {
      outspeed = 0;
    }
  }
  //若是倒车
  
}
//舵机限幅输出函数
void steer_out()
{
	//限幅
	if(outsteer>STEER_MAX)outsteer=STEER_MAX;
	if(outsteer<STEER_MIN)outsteer=STEER_MIN;
	//输出
	FTM_PWM_Duty(FTM3,FTM_CH6,outsteer);
}
//计算舵机输出
void steer_cal()
{
  if(((ADC2VAL>60) && (ADC1VAL<60) && (abs(ADC2VAL-ADC1VAL)>6) && (ADC3VAL>60) && Turn_Flag==0 && Turn_Right_Flag==0))
  {
    Turn_Left_Flag=0;//右
    Turn_Right_Flag=1;
  }
  if(((ADC1VAL>60) && (ADC2VAL<60) && (abs(ADC1VAL-ADC2VAL)>6) && (ADC3VAL>60) && Turn_Flag==0 && Turn_Left_Flag==0))
  {
    Turn_Left_Flag=1;//左
    Turn_Right_Flag=0;
  }
  if((ADC3VAL>85) && (ADC1VAL>55) && (ADC2VAL>55) && (abs(ADC1VAL-ADC2VAL)>10) && Turn_Flag==0 && (Turn_Left_Flag==1 || Turn_Right_Flag==1) && Go_Out_Circle==0)
  {
    Turn_Flag=1;
  }
  if((ADC3VAL<55) && ((ADC2VAL<45) || (ADC1VAL<45)) && Turn_Flag==1)
  {
    Turn_Flag=0;
    Turn_Right_Flag=0;
    Turn_Left_Flag=0;
    Go_Out_Circle=1;
    RoadType=2;
  }
  if(Go_Out_Circle==2)
  {
    Go_Out_Circle=0;
    Turn_Flag=0;
    Turn_Right_Flag=0;
    Turn_Left_Flag=0;
  }
  //判断道路情况，使用123电感值进行判断
  if((ADC1VAL>20)&&(ADC1VAL<40)&&(ADC2VAL>20)&&(ADC2VAL<40)&&(abs(ADC1VAL-ADC2VAL<10))&&(abs(ADC4VAL-ADC5VAL<10))&&(ADC3VAL<60))
  {
    RoadType=1;//直道
  }
  else
  {
    RoadType=2;
  }
  if(nowspeed>=1.2&&(ADC1VAL>20)&&(ADC1VAL<40)&&(ADC2VAL>20)&&(ADC2VAL<40)&&(abs(ADC1VAL-ADC2VAL<10))&&(abs(ADC4VAL-ADC5VAL<10))&&(ADC3VAL<60))
  {
    RoadType=3;//长直道
  }
  //开始计算偏差值
  dir_error = 100.0 * (AD2 - AD1) / (AD1 + AD2); 	        //更新偏差量
  //如果是弯道就跟着1+4 2+5的电感走
  if(RoadType==2)
  {
    dir_error=150*(AD2 + AD5 - AD1 - AD4)/(AD1 + AD2 + AD4 + AD5); 
  }
  //如果是环岛就跟着不同的电感走路
  if(Turn_Flag==1)   
  {
    if(Turn_Right_Flag==1)
    {
      dir_error=200*(AD2 - AD4)/(AD2 + AD4); 
    }
    if(Turn_Left_Flag==1)
    {
      dir_error=200*(AD5 - AD1)/(AD5 + AD1); 
    }  
  }
  
  //计算输出
  dir_error=dir_error*(dir_error*dir_error/1250.0+2)/10.0;										//使用公式对方向偏差量进行计算 
  outsteer = (int)(SET_ANGLE + 5 * DuoP * dir_error + DuoD * (dir_error - dir_old_error));//位置
  dir_old_error = dir_error;	//首先记录上一次的数据，接下来是判断道路状态并且求出偏差值
  //用于判断是不是需要激烈转弯
 if(Dir_last==2)  //原来在中间向左右偏离
  {
    if( (((ADC2VAL-ADC1VAL)>35)&&(Turn_Flag==0)&&(Turn_Left_Flag==0)&&(Turn_Right_Flag==0)) /*|| ((ADC2VAL<20)&&(ADC1VAL<20)&&((ADC5VAL - ADC4VAL)>10))*/ )  Dir_last = 1;   //上一状态为右转
    if( (((ADC1VAL-ADC2VAL)>35)&&(Turn_Flag==0)&&(Turn_Left_Flag==0)&&(Turn_Right_Flag==0)) /*|| ((ADC2VAL<20)&&(ADC1VAL<20)&&((ADC4VAL - ADC5VAL)>10))*/ )  Dir_last = 0;   //上一状态为左转
  }
  //大弯或丢线，左转
  if(Dir_last==1)  
  {
    outsteer = STEER_MAX;
  }
  //保持右转
  if(Dir_last==0)
  {
    outsteer = STEER_MIN;
  }
  Dir_last = 2;
  //判断是不是跑飞了
  if(ADC1VAL < 4 && ADC2VAL < 4 && outsteer_flag==0)
  {
    backcar = 1;
    outsteer_flag = outsteer>=SET_ANGLE?1:2;
  }
  if(backcar==0)
  {
    if(RoadType==1)
    {
      speed_set = 1.2;          //短直道1.2
    }
    if(RoadType==2)
    {
      speed_set = 1.0;          //弯道1.0
    }
    if(RoadType==3)
    {
      speed_set = 1.4;          //长直道1.2
    }
    backcar = 0;
  }
  if(outsteer_flag == 1) outsteer = STEER_MIN;
  if(outsteer_flag == 2) outsteer = STEER_MAX;
  if(outsteer_flag!=0 && (ADC1VAL >= 4 || ADC2VAL >= 4))
  {
    outsteer_flag=0;
    backcar = 0;
  }

}

void barrier_sr04()
{
  if(barrier_delay==1)
  {
    speed_set = 1.0;
  }
//超声波避障
 if(barrier_delay==2)
 {
   barrier = 1;
   speed_set = 1.0;
   outsteer= STEER_MIN;
 }
 if(barrier==1 && barrier_stage1==1)
 {
   outsteer= SET_ANGLE+450;
 }
 if(barrier==1 && barrier_stage2==1)
 {
   outsteer= STEER_MAX;
 }
 if(barrier==1 && barrier_stage3==1)
 { 
   barrier=0;
   barrier_stage1=0;
   barrier_stage2=0;
   barrier_stage3=0;
   barrier_delay=0;
 }
   
}
void ADCfilt()
{
	unsigned char filti;
	for(filti=0;filti<9;filti++)
	{
		ADC1filter[9-filti] = ADC1filter[8-filti];
	}
	ADC1filter[0] = ADC1VAL;
	for(filti=0;filti<9;filti++)
	{
		ADC2filter[9-filti] = ADC2filter[8-filti];
	}
	ADC2filter[0] = ADC2VAL;
	for(filti=0;filti<9;filti++)
	{
		ADC3filter[9-filti] = ADC3filter[8-filti];
	}
	ADC3filter[0] = ADC3VAL;
	for(filti=0;filti<9;filti++)
	{
		ADC4filter[9-filti] = ADC4filter[8-filti];
	}
	ADC4filter[0] = ADC4VAL;
	for(filti=0;filti<9;filti++)
	{
		ADC5filter[9-filti] = ADC5filter[8-filti];
	}
	ADC5filter[0] = ADC5VAL;
	for(filti=0;filti<9;filti++)
	{
		ADC6filter[9-filti] = ADC6filter[8-filti];
	}
	ADC6filter[0] = ADC6VAL;
	ADC1VAL = (ADC1filter[0]+ADC1filter[1]+ADC1filter[2]+ADC1filter[3]+ADC1filter[4]+ADC1filter[5]+ADC1filter[6]+ADC1filter[7]+ADC1filter[8]+ADC1filter[9])/10;
	ADC2VAL = (ADC2filter[0]+ADC2filter[1]+ADC2filter[2]+ADC2filter[3]+ADC2filter[4]+ADC2filter[5]+ADC2filter[6]+ADC2filter[7]+ADC2filter[8]+ADC2filter[9])/10;
	ADC3VAL = (ADC3filter[0]+ADC3filter[1]+ADC3filter[2]+ADC3filter[3]+ADC3filter[4]+ADC3filter[5]+ADC3filter[6]+ADC3filter[7]+ADC3filter[8]+ADC3filter[9])/10;
	ADC4VAL = (ADC4filter[0]+ADC4filter[1]+ADC4filter[2]+ADC4filter[3]+ADC4filter[4]+ADC4filter[5]+ADC4filter[6]+ADC4filter[7]+ADC4filter[8]+ADC4filter[9])/10;
	ADC5VAL = (ADC5filter[0]+ADC5filter[1]+ADC5filter[2]+ADC5filter[3]+ADC5filter[4]+ADC5filter[5]+ADC5filter[6]+ADC5filter[7]+ADC5filter[8]+ADC5filter[9])/10;
	ADC6VAL = (ADC6filter[0]+ADC6filter[1]+ADC6filter[2]+ADC6filter[3]+ADC6filter[4]+ADC6filter[5]+ADC6filter[6]+ADC6filter[7]+ADC6filter[8]+ADC6filter[9])/10;
}
void adval_get()
{
	//获取AD数值
	ADC1VAL = ADC_Ave(ADC_1,ADC1_SE14,ADC_12bit ,10);//AD1--SE14
	ADC2VAL = ADC_Ave(ADC_1,ADC1_SE15,ADC_12bit ,10);//AD2--SE15
	ADC3VAL = ADC_Ave(ADC_1,ADC1_SE10,ADC_12bit ,10);//AD3--SE10
	ADC4VAL = ADC_Ave(ADC_1,ADC1_SE13,ADC_12bit ,10);//AD4--SE13
	ADC5VAL = ADC_Ave(ADC_1,ADC1_SE12,ADC_12bit ,10);//AD5--SE12
	ADC6VAL = ADC_Ave(ADC_1,ADC1_SE11,ADC_12bit ,10);//AD6--SE11
	//限幅
	ADC1VAL = ADC1VAL>AD1MAX?AD1MAX:ADC1VAL;
	ADC2VAL = ADC2VAL>AD2MAX?AD2MAX:ADC2VAL;
	ADC3VAL = ADC3VAL>AD3MAX?AD3MAX:ADC3VAL;
	ADC4VAL = ADC4VAL>AD4MAX?AD4MAX:ADC4VAL;
	ADC5VAL = ADC5VAL>AD5MAX?AD5MAX:ADC5VAL;
	ADC6VAL = ADC6VAL>AD6MAX?AD6MAX:ADC6VAL;
	
	ADC1VAL = ADC1VAL<AD1MIN?AD1MIN:ADC1VAL;
	ADC2VAL = ADC2VAL<AD2MIN?AD2MIN:ADC2VAL;
	ADC3VAL = ADC3VAL<AD3MIN?AD3MIN:ADC3VAL;
	ADC4VAL = ADC4VAL<AD4MIN?AD4MIN:ADC4VAL;
	ADC5VAL = ADC5VAL<AD5MIN?AD5MIN:ADC5VAL;
	ADC6VAL = ADC6VAL<AD6MIN?AD6MIN:ADC6VAL;
	ADCfilt();
	//归一化
	//使用未归一化的数值
	AD1 = ADC1VAL;
	AD2 = ADC2VAL;
	AD3 = ADC3VAL;
	AD4 = ADC4VAL;
	AD5 = ADC5VAL;
	AD6 = ADC6VAL;
	ADC1VAL = 100*(ADC1VAL-AD1MIN)/AD1DIFF;
	ADC2VAL = 100*(ADC2VAL-AD2MIN)/AD2DIFF;
	ADC3VAL = 100*(ADC3VAL-AD3MIN)/AD3DIFF;
	ADC4VAL = 100*(ADC4VAL-AD4MIN)/AD4DIFF;
	ADC5VAL = 100*(ADC5VAL-AD5MIN)/AD5DIFF;
	ADC6VAL = 100*(ADC6VAL-AD6MIN)/AD6DIFF;
}
//获取小车的速度
void speed_get()
{
	FTMSPEED = -FTM_AB_Get(FTM2);     //获取正交解码速度，正负表示方向
	nowspeed = FTMSPEED/367.0; //147
}
//测距函数，每20ms测一次，每次运行这个函数，执行都会将上一次的超声波数值拿来使用
void sr04_use()
{
          GPIO_Ctrl(PTC,4,1);
          LPTMR_delay_us(20);
          GPIO_Ctrl(PTC,4,0);
}

float slope(unsigned int ydata[])
{
	float av_y; //声明变量
	float L_xy;
	int i = 0;
	//av_x = 2; //X的平均值
	av_y = 0; //Y的平均值
	//L_xx = 45; //Lxx
	L_xy = 0; //Lxy
	for(i = 0; i < 5; i++) //计算X、Y的平均值
	{
		av_y +=log((float)ydata[i]);;
	}
	av_y = av_y/5;
	for(i = 0; i < 5; i++) //计算Lxx、Lyy和Lxy
	{
		L_xy += (i-2)*(ydata[i]-av_y);
	}
	return L_xy/45;//计算斜率
}
void get_slope()
{
	int slope_i;
	for(slope_i=0;slope_i<3;slope_i++)
	{
		FSLOPE1[2-slope_i] = FSLOPE1[1-slope_i];
	}
	for(slope_i=0;slope_i<3;slope_i++)
	{
		FSLOPE2[2-slope_i] = FSLOPE2[1-slope_i];
	}
	for(slope_i=0;slope_i<3;slope_i++)
	{
		FSLOPE3[2-slope_i] = FSLOPE3[1-slope_i];
	}
	for(slope_i=0;slope_i<3;slope_i++)
	{
		FSLOPE4[2-slope_i] = FSLOPE4[1-slope_i];
	}
	for(slope_i=0;slope_i<3;slope_i++)
	{
		FSLOPE5[2-slope_i] = FSLOPE5[1-slope_i];
	}
	for(slope_i=0;slope_i<3;slope_i++)
	{
		FSLOPE6[2-slope_i] = FSLOPE6[1-slope_i];
	}
	FSLOPE1[0] = slope(ADC1filter);
	FSLOPE2[0] = slope(ADC2filter);
	FSLOPE3[0] = slope(ADC3filter);
	FSLOPE4[0] = slope(ADC4filter);
	FSLOPE5[0] = slope(ADC5filter);
	FSLOPE6[0] = slope(ADC6filter);
	SLOPE1 = (FSLOPE1[0] + FSLOPE1[1] + FSLOPE1[2])/3;
	SLOPE2 = (FSLOPE2[0] + FSLOPE2[1] + FSLOPE2[2])/3;
	SLOPE3 = (FSLOPE3[0] + FSLOPE3[1] + FSLOPE3[2])/3;
	SLOPE4 = (FSLOPE4[0] + FSLOPE4[1] + FSLOPE4[2])/3;
	SLOPE5 = (FSLOPE5[0] + FSLOPE5[1] + FSLOPE5[2])/3;
	SLOPE6 = (FSLOPE6[0] + FSLOPE6[1] + FSLOPE6[2])/3;
}