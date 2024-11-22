#include "include.h"

void LED_Init(void)
{
   GPIO_Init(PTC,18,GPO,1);
   GPIO_Init(PTC,19,GPO,1);
}

