#ifndef INCLUDE_H_
#define INCLUDE_H_

//ͨ��ͷ�ļ�
    #include    <stdio.h>                       //printf
    #include    <string.h>                      //memcpy
    #include    <stdlib.h>                      //malloc
		#include		<math.h>												//abs

//Cortex-M�ں�MCU�Ĵ���ͷ�ļ�
    #include "MK66F18.h"   //�Ĵ���ӳ��ͷ�ļ�
    #include "arm_math.h "
    #include "Systick.h"

//MCU�ڲ�ģ��������ͷ�ļ�
    #include "GPIO.h"
    #include "GPIO_Cfg.h"
    #include "PLL.h"
    #include "FTM.h"    
    #include "UART.h"
    #include "ADC.h"
    #include "PLL.h"    
    #include "PIT.h"
    #include "I2C.h"
    //#include "SPI.h"
    #include "DMA.h"
    #include "Lptmr.h"    
    #include "RTC.h"

//�ж��������жϺ�������
    #include "vectors.h"

//�������ͼ��˿������ض���
    #include "common.h"

//�ⲿ�豸���Զ��幦��������ͷ�ļ�
    //#include "LQ9AX.h"
    #include "LED.h"
    #include "KEY.h"
    #include "OLED.h"
		#include "debug.h"
		#include "control.h"

#endif