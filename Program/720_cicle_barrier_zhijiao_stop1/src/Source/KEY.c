#include "include.h"


void KEY_Init(void)
{
   GPIO_Init(PTE,2,GPI,1);
   GPIO_Init(PTE,3,GPI,1);
   GPIO_Init(PTE,4,GPI,1);   
}


u8 KEY_Read(KEYn_e keyno)
{
    switch(keyno) 
    {
      case KEY0:
        return GPIO_Get(PTE4);
      break;
      
      case KEY1:
        return GPIO_Get(PTE2);
      break;
      
      case KEY2:
        return GPIO_Get(PTE3);
      break;
      default:
        return 0XFF;
           
    }
}
void SR04_init()
{
	GPIO_Init(PTC,4,GPO, 0); 
	
	EXTI_Init(PTC,5,either_down);
	 /* ���ȼ����� ��ռ���ȼ�1  �����ȼ�2   ԽС���ȼ�Խ��  ��ռ���ȼ��ɴ�ϱ���ж� */
	NVIC_SetPriority(PORTC_IRQn,NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0,0));
	NVIC_EnableIRQ(PORTC_IRQn);			         //ʹ��PORTE_IRQn���ж� 
        pit_time_start(PIT1);	//��ʼ����ʱ������ʹ���ж�
	//GPIO_Ctrl(PTC,4,1);
}


