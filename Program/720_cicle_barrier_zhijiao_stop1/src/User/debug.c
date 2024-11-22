#include "include.h" 


//
extern float SLOPE1 ;
extern float SLOPE2 ;
extern float SLOPE3 ;
extern float SLOPE4 ;
extern float SLOPE5 ;
extern float SLOPE6 ;
//声明所需要的变量
extern unsigned char backcar;
extern unsigned char stopcar;
extern unsigned char stopmotor;
extern unsigned char rightflag;
extern unsigned char leftflag;
extern unsigned char turnflag;
//AD值
extern int ADC1VAL;
extern int ADC2VAL;
extern int ADC3VAL;
extern int ADC4VAL;
extern int ADC5VAL;
extern int ADC6VAL;
//当前车速
extern volatile uint32_t hc_time;
extern int distance;
extern int FTMSPEED;
extern float nowspeed;
extern  float dir_error;
//用于显示的电机输出和舵机输出数据
extern volatile int outspeed;
extern volatile int outsteer;
//蓝牙调试标志位
extern int bluedebug;
//oled调试标志位
extern int oleddebug;
//串口接受判定数据
extern char recdatabuff[60];
extern char rec;
//用于显示的数据
extern float speed_Kp;
extern float speed_Ki;
extern float speed_Kd;
extern float speed_set;
extern unsigned char outsteer_flag;
//串口发送数据
unsigned char showbuff[20] = "";
char sendbuff[60] = "please inter password\r\n";	//初始化命令
//环岛标志位
extern unsigned char Turn_Left_Flag;
extern unsigned char Turn_Right_Flag;
extern unsigned char Turn_Flag;
extern unsigned char Go_Out_Circle;
//避障标志位
extern unsigned char barrier_delay;
//用于显示道路类型
extern unsigned char RoadType;
extern unsigned char backcar;
extern unsigned char stopcar;
//
extern int flag1;
extern int flag2;
extern int flag3;
void info_show()
{
  sprintf((char *)showbuff,"%5.3f %3d",nowspeed,outsteer);
  LCD_P6x8Str(0,6,showbuff);
  sprintf((char *)showbuff,"%3d % 3.1f",ADC1VAL,SLOPE1);
  LCD_P6x8Str(0,0,showbuff);
  sprintf((char *)showbuff,"%3d % 3.1f",ADC2VAL,SLOPE2);
  LCD_P6x8Str(0,1,showbuff);
  sprintf((char *)showbuff,"%3d % 3.1f",ADC4VAL,SLOPE4);
  LCD_P6x8Str(0,2,showbuff);
  sprintf((char *)showbuff,"%3d % 3.1f",ADC5VAL,SLOPE5);
  LCD_P6x8Str(0,3,showbuff);
  sprintf((char *)showbuff,"%3d % 3.1f",ADC3VAL,SLOPE3);
  LCD_P6x8Str(0,4,showbuff);
  sprintf((char *)showbuff,"%3d % 3.1f % 6.1f",ADC6VAL,SLOPE6,dir_error);
  LCD_P6x8Str(0,5,showbuff);
  
  sprintf((char *)showbuff," %1d",Turn_Left_Flag);
  LCD_P6x8Str(60,1,showbuff);
  sprintf((char *)showbuff," %1d",Turn_Right_Flag);
  LCD_P6x8Str(60,2,showbuff);
  sprintf((char *)showbuff," %1d",Turn_Flag);
  LCD_P6x8Str(60,3,showbuff);
  sprintf((char *)showbuff," %1d",Go_Out_Circle);
  LCD_P6x8Str(60,4,showbuff);
  sprintf((char *)showbuff,"%1d",RoadType);
  LCD_P6x8Str(80,0,showbuff);
  sprintf((char *)showbuff,"%1d",backcar);
  LCD_P6x8Str(80,1,showbuff);        
  sprintf((char *)showbuff,"%6d",distance);
  LCD_P6x8Str(80,2,showbuff);       
  sprintf((char *)showbuff,"%7d",barrier_delay);
  LCD_P6x8Str(80,3,showbuff);   
}
void blue_debug()
{
  
}
void key_scan()
{
  //用于开启OLED显示
  if(!KEY_Read(KEY1))           //如果KEY1按下 
  {
    if(!KEY_Read(KEY1))
    {	
       
    }
    while(!KEY_Read(KEY1));
  }
  else if(!KEY_Read(KEY2))     //如果KEY2按下
  { 
    if(!KEY_Read(KEY2))
    {
    }
    while(!KEY_Read(KEY2));
  }
  else if(!KEY_Read(KEY0))     //如果KEY0按下
  {
    if(!KEY_Read(KEY0))
    {	
      if(stopmotor==1)	
      {
        stopmotor = 2;
      }
      if(stopmotor==0)	
      {
        stopmotor = 1;
        stopcar = 1;
      }
      if(stopmotor==2)	
      {
        stopmotor = 0;
        stopcar = 0;
      }
    }
    while(!KEY_Read(KEY0));
  }
}
void send_info()
{
  //发送AD数值 
  //使用下面的数据格式
  //AXXXBXXXCXXXDXXXEXXXFXGXHXIXJXPXX.XXXIXX.XXXDXX.XXX
  //对于整数就是%d，无符号字符型就是强制类型转换加%d，浮点型数据就是%5.3f
  //AD的数值就是		ADC1VAL ADC2VAL ADC3VAL ADC4VAL ADC5VAL
  //标志位就是，现在只有两个可以添加
  //sprintf(sendbuff,"A%3dB%3dC%3dD%3dE%3dF%1dG%1dH%1dI%1dJ%1dP%6.3fI%6.3fD%6.3fT%6.3f",ADC1VAL,ADC2VAL,ADC3VAL,ADC4VAL,ADC5VAL,bluedebug,oleddebug,flag1,flag2,flag3,speed_Kp,speed_Ki,speed_Kd,speed_set);
  //UART_Put_Str(UART_4,(unsigned char *)sendbuff);
  //发送速度数值
  //显示参数
  //显示标志位
}
void data_receive()
{
  if(recdatabuff[0]=='F')
  {
    //sscanf(recdatabuff,"F%dG%dH%dI%dJ%dP%fI%fD%fT%f",&bluedebug,&oleddebug,&flag1,&flag2,&flag3,&speed_Kp,&speed_Ki,&speed_Kd,&speed_set);
    //recdatabuff[0]='\0';
  }
}


