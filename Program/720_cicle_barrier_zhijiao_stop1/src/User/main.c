//ʹ��FLASH�ļ�link��ʹ�������ļ�����
//C�Ż��ȼ�λLOW
//���PWM�ڵĶ���FTM.H��
//#define FTM3_CH6    PTC10       // PTE11��PTC10
//��������ڵĶ�����FTM.H��
//���������
//#define FTM2_QDPHA  PTB18       //PTA10��PTB18
//#define FTM2_QDPHB  PTB19       //PTA11��PTB19
//ʹ�ð���������λ������״̬����λ�����ݲ���֡ͷ֡β�����ƺ���д��debug�ļ�
//ADCͨ����������ADC.H�ļ��ж���
//ADC1_SE14 =14,      // PTB10
//ADC1_SE15 =15,      // PTB11
//ADC1_SE10 =10,      // PTB4
//ADC1_SE11 =11,      // PTB5
//ADC1_SE12 =12,      // PTB6
//���PWM�ڵĶ���FTM.H��
//#define FTM0_CH0    PTC1        //PTC1��PTA3            PTA3��Ҫ�ã���JLINK��ͻ��
//#define FTM0_CH1    PTC2        //PTC2��PTA4
//���PWM�ڵĶ���UART.H��
//#define UART4_RX    PTE25       //PTC14��PTE25
//#define UART4_TX    PTE24       //PTC15��PTE24

#include "include.h" 

/*******************************ȫ�ֱ���***************************************/
//PIT��ʱ��ֵ
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
//���ϱ�־λ
unsigned char barrier = 0;
unsigned char barrier_delay = 0;
unsigned char ba_delay1S = 0;
unsigned int ba_timer = 0; 
unsigned char barrier_stage1 = 0; 
unsigned char barrier_stage2 = 0;
unsigned char barrier_stage3 = 0; 
volatile uint32_t hc_time = 0;
int distance = 300;
//���ڴ洢AD��ֵ
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
//���ڴ洢�ٶ�ֵ
int FTMSPEED=0;
float nowspeed=0;
//���򻷵�����
float dir_error = 0;
float dir_old_error = 0;
float DuoP = 5.0;
float DuoD = 1.5;
//����Ͷ�������ֵ
volatile int outspeed = 0;
volatile int outsteer = SET_ANGLE;
//���ڵ��Ե���ر�־λ
int bluedebug = 1;
int oleddebug = 1;
//������������
char recflag[7] = "";	//�����ж���ʼ����
char rec = '\0';			//���������ж�
unsigned char decnum = 0;
char recdatabuff[60] = "";
//ͣ����־λ
unsigned char stopflag = 0;
//���ڷ���������ر�־λ�ͱ���
unsigned char send_sec = 0;//��ʱ��������
//���Բ�����ر�־λ
int flag1 = 0;
int flag2 = 0;
int flag3 = 0;
//������ر�־λ

//�ٶȻ�PID
float speed_set = 1.0;
float speed_error = 0;
float speed_error_last = 0;
float speed_error_last_last = 0;
float speed_Kp = 30;  //7.6
float speed_Ki = 0.0;
float speed_Kd = 1.5;  //0.75


/*****************************************************************************/
/*******************************��������***************************************/
extern void UART_Put_Char (UARTn_e uratn, char ch);
/*****************************************************************************/
/*******************************������****************************************/

 void main(void)
{
  DisableInterrupts;        //�ر����ж�
  PLL_Init(PLL200);         //��ʼ��PLLΪ200M������Ϊ40MHZ  
  NVIC_SetPriorityGrouping(0x07 - 2);	//�����ж����ȼ�����         
  SR04_init();
  LED_Init();               //LED��ʼ��
  LCD_Init();               //OLED��ʾ��ʼ��
  KEY_Init();               //������ʼ��
  UART_Init(UART_4,115200); //����4��ʼ��  TX E24  RX E25
  //����ӿڳ�ʼ��
  FTM_PWM_Init(FTM3,FTM_CH6,50000,SET_ANGLE);//Ƶ�ʣ�40M/(2^4)/50000=50HZ,ÿ������Ϊ0.2us PTC10
  //����ӿڳ�ʼ��
  FTM_PWM_Init(FTM0,FTM_CH0,4166,0);//Ƶ�ʣ�40M/(2^3)/4166=1200HZ    PTC1
  FTM_PWM_Init(FTM0,FTM_CH1,4166,0);//Ƶ�ʣ�40M/(2^3)/4144=1200HZ	 PTC2
  //ADC��ʼ��
  ADC_Init(ADC_1);
  //ͣ����ʼ��
  GPIO_Init(PTD,10,GPO,1);//��ʼ��Ϊ1
  EXTI_Init(PTD,10,falling_up);
  //�������ӿڳ�ʼ��
  FTM_AB_Init(FTM2);        //������������ PTB18 PTB19
  EnableInterrupts;					//ʹ�����ж�
  UART_Irq_En(UART_4);			//��������4�����ж�
  PIT_Init(PIT0, 10);      	//��ʼ������PIT�жϣ�10ms
  NVIC_SetPriority(PIT0_IRQn,NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1,1));
  NVIC_EnableIRQ(PORTC_IRQn);	
	outspeed = 0;						//���������ʼ��
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
/*******************************�жϺ���***************************************/
//PIT�ж϶�ʱ�����ٶ�ֵ
void PIT0_Interrupt()
{
  PIT_Flag_Clear(PIT0);       //���жϱ�־λ 
  //������ʱ
  if(Go_Out_Circle==1)
  {
    cirflag8S++;
  }
  if(cirflag8S>=800)
  {
    cirflag8S = 0;
    Go_Out_Circle = 2;
  }
  //���ϼ�ʱ
   flag20ms++;
  if(flag20ms>2)                          
  {
    flag20ms = 0;
    if(barrier_delay==1 && distance<=210&& (ADC1VAL>20) && (ADC1VAL<50) && (ADC2VAL>20) && (ADC2VAL<50) && (abs(ADC1VAL-ADC2VAL<20)) && (abs(ADC4VAL-ADC5VAL<25)) && (ADC3VAL<60))
    {
      barrier_delay = 2;
    }
    if(distance<=260 && barrier_delay==0 && (ADC1VAL>20) && (ADC1VAL<50) && (ADC2VAL>20) && (ADC2VAL<50) && (abs(ADC1VAL-ADC2VAL<20)) && (abs(ADC4VAL-ADC5VAL<25)) && (ADC3VAL<60))       //����2���ж�
    {
      barrier_delay = 1;  
    }
    sr04_use();          //ÿ20ms����������һ�δ����ź�
  }
  if(barrier_delay==1)   //���ϱ�־λ��ʱ1s����
  {
    ba_timer1S++;
  } 
  if(ba_timer1S>=10)
  {
    ba_timer1S = 0;
    if(barrier_delay!=2) barrier_delay=0;
  }
  if(barrier==1)                 //������ʱ
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
//ͣ����ʱ
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
  motor_out();//����Ͷ����ֵ
  speed_get();
  adval_get();//��ȡAD��ֵ
//  get_ slope();
  if(barrier_delay==0)
  {
    steer_cal();//���������������ֵ   
  }
  steer_out(); 
}
//�жϽ��ܺ���
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
    if((GPIO_Get(PTC5)==1)&&(sr04start==0))//������
    {
      PIT->CHANNEL[PIT1].TCTRL  = ( 0 | PIT_TCTRL_TEN_MASK );      //������ʱ��
      sr04start = 1;
    }
    if((GPIO_Get(PTC5)==0)&&(sr04start==1))//�½���
    {
      hc_time = pit_time_get(PIT1);                           //��ö�ʱ�����ݲ��ҹص���ʱ��
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
  PIT_Flag_Clear(PIT1);       //���жϱ�־λ
  /*�û�����������*/
}
/*****************************************************************************/

