//使用FLASH文件link，使用内置文件下载
//C优化等级位LOW
//舵机PWM口的定义FTM.H中
//#define FTM3_CH6    PTC10       // PTE11、PTC10
//正交编码口的定义在FTM.H中
//正交编码二
//#define FTM2_QDPHA  PTB18       //PTA10、PTB18
//#define FTM2_QDPHB  PTB19       //PTA11、PTB19
//使用按键设置上位机调试状态，上位机数据不加帧头帧尾，控制函数写在debug文件
//ADC通道和引脚在ADC.H文件中定义
//ADC1_SE14 =14,      // PTB10
//ADC1_SE15 =15,      // PTB11
//ADC1_SE10 =10,      // PTB4
//ADC1_SE11 =11,      // PTB5
//ADC1_SE12 =12,      // PTB6
//电机PWM口的定义FTM.H中
//#define FTM0_CH0    PTC1        //PTC1、PTA3            PTA3不要用（与JLINK冲突）
//#define FTM0_CH1    PTC2        //PTC2、PTA4
//电机PWM口的定义UART.H中
//#define UART4_RX    PTE25       //PTC14、PTE25
//#define UART4_TX    PTE24       //PTC15、PTE24

#include "include.h" 

/*******************************全局变量***************************************/
//PIT计时标值
unsigned int flag20ms = 0;
unsigned int cirflag8S = 0;
unsigned int ba_timer1S = 0;
unsigned int stop8S = 0;
//
extern unsigned char Go_Out_Circle;
unsigned char backcar = 0;
unsigned char stopcar = 1;
unsigned char stopmotor = 1;
unsigned char rightflag = 0;
unsigned char leftflag = 0;
unsigned char turnflag = 0;
//
uint8_t sr04start = 0;
//避障标志位
unsigned char barrier = 0;
unsigned char barrier_delay = 0;
unsigned char ba_delay1S = 0;
unsigned int ba_timer = 0; 
unsigned char barrier_stage1 = 0; 
unsigned char barrier_stage2 = 0;
unsigned char barrier_stage3 = 0; 
volatile uint32_t hc_time = 0;
int distance = 300;
//用于存储AD数值
int AD1 = 0;
int AD2 = 0;
int AD3 = 0;
int AD4 = 0;
int AD5 = 0;
int AD6 = 0;
int ADC1VAL = 0;
int ADC2VAL = 0;
int ADC3VAL = 0;
int ADC4VAL = 0;
int ADC5VAL = 0;
int ADC6VAL = 0;
//用于存储速度值
int FTMSPEED=0;
float nowspeed=0;
//方向环的数据
float dir_error = 0;
float dir_old_error = 0;
float DuoP = 5.0;
float DuoD = 1.5;
//电机和舵机的输出值
volatile int outspeed = 0;
volatile int outsteer = SET_ANGLE;
//用于调试的相关标志位
int bluedebug = 1;
int oleddebug = 1;
//蓝牙接受数据
char recflag[7] = "";	//用于判定起始数据
char rec = '\0';			//用于数据判定
unsigned char decnum = 0;
char recdatabuff[60] = "";
//停车标志位
unsigned char stopflag = 0;
//串口发送数据相关标志位和变量
unsigned char send_sec = 0;//计时发送数据
//调试参数相关标志位
int flag1 = 0;
int flag2 = 0;
int flag3 = 0;
//运行相关标志位

//速度环PID
float speed_set = 1.0;
float speed_error = 0;
float speed_error_last = 0;
float speed_error_last_last = 0;
float speed_Kp = 30;  //7.6
float speed_Ki = 0.0;
float speed_Kd = 1.5;  //0.75


/*****************************************************************************/
/*******************************声明函数***************************************/
extern void UART_Put_Char (UARTn_e uratn, char ch);
/*****************************************************************************/
/*******************************主函数****************************************/

 void main(void)
{
  DisableInterrupts;        //关闭总中断
  PLL_Init(PLL200);         //初始化PLL为200M，总线为40MHZ  
  NVIC_SetPriorityGrouping(0x07 - 2);	//设置中断优先级分组         
  SR04_init();
  LED_Init();               //LED初始化
  LCD_Init();               //OLED显示初始化
  KEY_Init();               //按键初始化
  UART_Init(UART_4,115200); //串口4初始化  TX E24  RX E25
  //舵机接口初始化
  FTM_PWM_Init(FTM3,FTM_CH6,50000,SET_ANGLE);//频率：40M/(2^4)/50000=50HZ,每个脉冲为0.2us PTC10
  //电机接口初始化
  FTM_PWM_Init(FTM0,FTM_CH0,4166,0);//频率：40M/(2^3)/4166=1200HZ    PTC1
  FTM_PWM_Init(FTM0,FTM_CH1,4166,0);//频率：40M/(2^3)/4144=1200HZ	 PTC2
  //ADC初始化
  ADC_Init(ADC_1);
  //停车初始化
  GPIO_Init(PTD,10,GPO,1);//初始化为1
  EXTI_Init(PTD,10,falling_up);
  //编码器接口初始化
  FTM_AB_Init(FTM2);        //开启正交解码 PTB18 PTB19
  EnableInterrupts;					//使能总中断
  UART_Irq_En(UART_4);			//开启串口4接收中断
  PIT_Init(PIT0, 10);      	//初始化开启PIT中断，10ms
  NVIC_SetPriority(PIT0_IRQn,NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1,1));
  NVIC_EnableIRQ(PORTC_IRQn);	
	outspeed = 0;						//电机参数初始化
  while(1)
  {
		key_scan();
		if(oleddebug)
		{
			info_show();
		}
		if(bluedebug)
		{
			blue_debug();
		}
		if(send_sec>50)
		{
			send_sec = 0;
			send_info();
			data_receive();
		}
   }  
}
/*****************************************************************************/
/*******************************中断函数***************************************/
//PIT中断定时接受速度值
void PIT0_Interrupt()
{
  PIT_Flag_Clear(PIT0);       //清中断标志位 
  //环岛计时
  if(Go_Out_Circle==1)
  {
    cirflag8S++;
  }
  if(cirflag8S>=800)
  {
    cirflag8S = 0;
    Go_Out_Circle = 2;
  }
  //避障计时
   flag20ms++;
  if(flag20ms>2)                          
  {
    flag20ms = 0;
    if(barrier_delay==1 && distance<=210&& (ADC1VAL>20) && (ADC1VAL<50) && (ADC2VAL>20) && (ADC2VAL<50) && (abs(ADC1VAL-ADC2VAL<20)) && (abs(ADC4VAL-ADC5VAL<25)) && (ADC3VAL<60))
    {
      barrier_delay = 2;
    }
    if(distance<=260 && barrier_delay==0 && (ADC1VAL>20) && (ADC1VAL<50) && (ADC2VAL>20) && (ADC2VAL<50) && (abs(ADC1VAL-ADC2VAL<20)) && (abs(ADC4VAL-ADC5VAL<25)) && (ADC3VAL<60))       //进行2次判断
    {
      barrier_delay = 1;  
    }
    sr04_use();          //每20ms超声波发送一次触发信号
  }
  if(barrier_delay==1)   //避障标志位定时1s清零
  {
    ba_timer1S++;
  } 
  if(ba_timer1S>=10)
  {
    ba_timer1S = 0;
    if(barrier_delay!=2) barrier_delay=0;
  }
  if(barrier==1)                 //避障延时
  {
    ba_timer++;
  }
  if(ba_timer>=90)
  {
    barrier_stage1 = 1;
  }
  if(ba_timer>=165)
  {
    barrier_stage2 = 1;
  }
  if(ba_timer>=240)
  {
    ba_timer = 0;
    barrier_stage3 = 1;
  }
//停车计时
  if(stopflag==0&&stopcar==0)
  {
    stop8S++;
  }
  if(stop8S>=800 && stopflag==0)
  {
    stopflag=1;
    stop8S=0;
  }
    
  barrier_sr04();
  send_sec++;
  
  motor_cal();//
  motor_out();//电机和舵机赋值
  speed_get();
  adval_get();//获取AD数值
//  get_ slope();
  if(barrier_delay==0)
  {
    steer_cal();//根据情况计算舵机数值   
  }
  steer_out(); 
}
//中断接受函数
void UART4_IRQHandler(void)
{
  recdatabuff[decnum] = UART_Get_Char (UART_4);
  decnum++;
  if(decnum>=38)	decnum=0;
}
//
void PORTC_Interrupt()
{
  if((PORTC_ISFR & (1<<5)))
  {
    PORTC_ISFR |= (1<<5);
    if((GPIO_Get(PTC5)==1)&&(sr04start==0))//上升沿
    {
      PIT->CHANNEL[PIT1].TCTRL  = ( 0 | PIT_TCTRL_TEN_MASK );      //开启定时器
      sr04start = 1;
    }
    if((GPIO_Get(PTC5)==0)&&(sr04start==1))//下降沿
    {
      hc_time = pit_time_get(PIT1);                           //获得定时器数据并且关掉定时器
      if(distance>290 && hc_time/1000<220);
      else distance = hc_time/1000;
      sr04start = 0;
    }
  }  
}

void PORTD_Interrupt()
{
  if((PORTD_ISFR & (1<<10)))
  {
      PORTD_ISFR |= (1<<10);
      GPIO_Ctrl(PTD,10,1);      
      if(stopflag==1)
      {
        stopcar = 1;
        stopflag = 0;
      }
  }
}

void PIT1_Interrupt()
{
  PIT_Flag_Clear(PIT1);       //清中断标志位
  /*用户添加所需代码*/
}
/*****************************************************************************/

