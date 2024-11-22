/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
��ƽ    ̨�������������ܿƼ�MK66FX1M0VLQ18���İ�
����    д��CHIUSIR
����    ע��
������汾��V1.0
�������¡�2016��08��20��
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://shop36265907.taobao.com
���������䡿chiusir@163.com
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include "include.h"
#include "PIT.h"

//-------------------------------------------------------------------------*
//������: pit_init
//��  ��: ��ʼ��PIT
//��  ��: pitn:ģ����PIT0��PIT1��PIT2��PIT3
//        cnt �ж�ʱ�䣬��λ1ms
//��  ��: ��
//��  ��: pit_init(PIT0,1000); PIT0�жϣ�1000ms����1s����PIT0_interrupt()һ��
//-------------------------------------------------------------------------*
void PIT_Init(PITn pitn, u32 cnt)
{
    //PIT �õ��� Bus Clock ����Ƶ��

    /* ����ʱ��*/
    SIM_SCGC6       |= SIM_SCGC6_PIT_MASK;                            //ʹ��PITʱ��

    /* PITģ����� PIT Module Control Register (PIT_MCR) */
    PIT_MCR         &= ~(PIT_MCR_MDIS_MASK | PIT_MCR_FRZ_MASK );      //ʹ��PIT��ʱ��ʱ�� ������ģʽ�¼�������

    /* ��ʱ������ֵ���� Timer Load Value Register (PIT_LDVALn) */
    PIT_LDVAL(pitn)  = cnt*40*1000;                                          //��������ж�ʱ��

    //��ʱʱ�䵽�˺�TIF �� 1 ��д1��ʱ��ͻ���0
    PIT_Flag_Clear(pitn);                                             //���жϱ�־λ

    /* ��ʱ�����ƼĴ��� Timer Control Register (PIT_TCTRL0) */
    PIT_TCTRL(pitn) |= ( PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK );   //ʹ�� PITn��ʱ��,����PITn�ж�

    NVIC_EnableIRQ((IRQn_Type)(pitn + 48));			                                //���������ŵ�IRQ�ж�
}

void pit_time_start(PITn pitn)
{
	SIM->SCGC6       |= SIM_SCGC6_PIT_MASK;        			//ʹ��PITʱ��
	PIT->MCR         = 0;									//ʹ��PIT��ʱ��ʱ�� ������ģʽ�¼�������
	PIT->CHANNEL[pitn].LDVAL  = ~0;              			//��������ж�ʱ��
	PIT_FlAG_CLR(pitn);										//����жϱ�־λ
	PIT->CHANNEL[pitn].TCTRL &= ~ PIT_TCTRL_TEN_MASK;       //��ֹPITn��ʱ����������ռ���ֵ��
  /*PIT->CHANNEL[pitn].TCTRL  = ( 0
								| PIT_TCTRL_TEN_MASK        //ʹ�� PITn��ʱ��
								//| PIT_TCTRL_TIE_MASK        //��PITn�ж�
								);*/
}
uint32 pit_time_get(PITn pitn)
{
	uint32 val;
	val = 0xFFFFFFFF - PIT->CHANNEL[pitn].CVAL;
        if(val>300000) val = 300000;
	if(PIT->CHANNEL[pitn].TFLG &  PIT_TFLG_TIF_MASK)		//�ж�ʱ���Ƿ�ʱ
	{
          val = 300000;
	}
        PIT_FlAG_CLR(pitn);									//����жϱ�־λ
	PIT->CHANNEL[pitn].TCTRL &= ~ PIT_TCTRL_TEN_MASK;   //��ֹPITn��ʱ����������ռ���ֵ��
		
      return val;
}
//-------------------------------------------------------------------------*
//������: PIT0_interrupt
//��  ��: PIT�жϺ���
//��  ��: ��
//��  ��: ��
//��  ��: �ɳ�ʼ���������೤ʱ�����һ��
//-------------------------------------------------------------------------*
//void PIT0_Interrupt()
//{
//  PIT_Flag_Clear(PIT0);       //���жϱ�־λ
 /*�û�����������*/
  //LED_Ctrl(LED2, RVS);        //�жϷ�����LED��˸
  //speed=FTM_AB_Get(FTM2);     //������������󣬿��Ի�ȡ�ٶȣ�������ʾ����

//}
void PIT2_Interrupt()
{
  PIT_Flag_Clear(PIT2);       //���жϱ�־λ
  /*�û�����������*/
}

void PIT3_Interrupt()
{
  PIT_Flag_Clear(PIT3);       //���жϱ�־λ
  /*�û�����������*/
}