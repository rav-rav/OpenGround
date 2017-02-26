/*
 Copyright 2016 fishpepper <AT> gmail.com

 This program is free software: you can redistribute it and/ or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http:// www.gnu.org/ licenses/>.

 author: fishpepper <AT> gmail.com
 */

#include "main.h"
#include <string.h>
#include <stdio.h>
#include "multi4in1.h"
#include "debug.h"
#include "config.h"

#include "stm32f0xx_rcc.h"
#include "stm32f0xx_usart.h"

static int16_t counter = 0;

uint16_t calcClock(USART_TypeDef* USARTx, USART_InitTypeDef* USART_InitStruct, uint32_t apbclock) {
    uint32_t divider = 0, tmpreg = 0;
    /* Determine the integer part */
    if ((USARTx->CR1 & USART_CR1_OVER8) != 0)
    {
      /* (divider * 10) computing in case Oversampling mode is 8 Samples */
      divider = (uint32_t)((2 * apbclock) / (USART_InitStruct->USART_BaudRate));
      tmpreg  = (uint32_t)((2 * apbclock) % (USART_InitStruct->USART_BaudRate));
    }
    else /* if ((USARTx->CR1 & CR1_OVER8_Set) == 0) */
    {
      /* (divider * 10) computing in case Oversampling mode is 16 Samples */
      divider = (uint32_t)((apbclock) / (USART_InitStruct->USART_BaudRate));
      tmpreg  = (uint32_t)((apbclock) % (USART_InitStruct->USART_BaudRate));
    }

    /* round the divider : if fractional part i greater than 0.5 increment divider */
    if (tmpreg >=  (USART_InitStruct->USART_BaudRate) / 2)
    {
      divider++;
    }

    /* Implement the divider in case Oversampling mode is 8 Samples */
    if ((USARTx->CR1 & USART_CR1_OVER8) != 0)
    {
      /* get the LSB of divider and shift it to the right by 1 bit */
      tmpreg = (divider & (uint16_t)0x000F) >> 1;

      /* update the divider value */
      divider = (divider & (uint16_t)0xFFF0) | tmpreg;
    }

    debug_put_hex32(apbclock);
    debug(" ");
    debug(" div ");
    debug_put_uint16((uint16_t)divider);
    debug_put_newline();

    /* Write to USART BRR */
    return (uint16_t)divider;
}

void SCCU (void)
{
  uint32_t tmp = 0, pllmull = 0, pllsource = 0, prediv1factor = 0;
  uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};

  /* Get SYSCLK source -------------------------------------------------------*/
  tmp = RCC->CFGR & RCC_CFGR_SWS;

  switch (tmp)
  {
    case 0x00:  /* HSI used as system clock */
      SystemCoreClock = HSI_VALUE;
      break;
    case 0x04:  /* HSE used as system clock */
      SystemCoreClock = HSE_VALUE;
      break;
    case 0x08:  /* PLL used as system clock */
      /* Get PLL clock source and multiplication factor ----------------------*/
      pllmull = RCC->CFGR & RCC_CFGR_PLLMULL;
      pllsource = RCC->CFGR & RCC_CFGR_PLLSRC;
      pllmull = ( pllmull >> 18) + 2;

      if (pllsource == 0x00)
      {
        /* HSI oscillator clock divided by 2 selected as PLL clock entry */
        SystemCoreClock = (HSI_VALUE >> 1) * pllmull;
      }
      else
      {
        prediv1factor = (RCC->CFGR2 & RCC_CFGR2_PREDIV1) + 1;
        /* HSE oscillator clock selected as PREDIV1 clock entry */
        SystemCoreClock = (HSE_VALUE / prediv1factor) * pllmull;
        debug("SCCU0 ");
        debug_put_hex32((HSE_VALUE / prediv1factor) * pllmull);
        debug("SCCU1 ");
        debug_put_hex32(SystemCoreClock);
      }
      break;
    default: /* HSI used as system clock */
      SystemCoreClock = HSI_VALUE;
      break;
  }
  /* Compute HCLK clock frequency ----------------*/
  /* Get HCLK prescaler */
  tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
  /* HCLK clock frequency */
  SystemCoreClock >>= tmp;
    debug("SCCU2 ");
    debug_put_hex32(SystemCoreClock);
}

void multi4in1_init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    USART_ClockInitTypeDef USART_ClockInitstructure;

    debug("multi4in1: set clocks\n");
    debug_flush();

    // Turn on USART1
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    // Turn on IO Port C
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

    // Configure USART3 Tx as alternate function push-pull
    debug("multi4in1: set pin config\n");
    debug_flush();
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource4, GPIO_AF_1);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource5, GPIO_AF_1);

    RCC_USARTCLKConfig(RCC_USART3CLK_HSI);
    debug("multi4in1: set usart config\n");
    debug_flush();
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);

    USART_ClockInitstructure.USART_Clock = USART_Clock_Disable;
    USART_ClockInitstructure.USART_CPOL = USART_CPOL_Low;
    USART_ClockInitstructure.USART_LastBit = USART_LastBit_Enable;
    USART_ClockInitstructure.USART_CPHA = USART_CPHA_1Edge;
    USART_Init(USART3, &USART_InitStructure);

    debug("multi4in1: enable usart\n");

    //USART_SetPrescaler(USART3, uint8_t USART_Prescaler);


    RCC_ClocksTypeDef RCC_ClocksStatus;
    RCC_GetClocksFreq(&RCC_ClocksStatus);
    debug("SYSCLK ");
    calcClock(USART3, &USART_InitStructure, RCC_ClocksStatus.SYSCLK_Frequency);
    //debug("USART1CLK ");
    //calcClock(USART1, &USART_InitStructure, RCC_ClocksStatus.USART1CLK_Frequency);
    //debug("USART2CLK ");
    //calcClock(USART2, &USART_InitStructure, RCC_ClocksStatus.USART2CLK_Frequency);
    debug("USART3CLK ");
    calcClock(USART3, &USART_InitStructure, RCC_ClocksStatus.USART3CLK_Frequency);

    SCCU();
    debug(" SystemCoreClock ");
    debug_put_hex32(SystemCoreClock);

    SystemCoreClockUpdate();
    debug(" SystemCoreClock ");
    debug_put_hex32(SystemCoreClock);

    RCC_GetClocksFreq(&RCC_ClocksStatus);
    debug("SYSCLK ");
    calcClock(USART3, &USART_InitStructure, RCC_ClocksStatus.SYSCLK_Frequency);
    //debug("USART1CLK ");
    //calcClock(USART1, &USART_InitStructure, RCC_ClocksStatus.USART1CLK_Frequency);
    //debug("USART2CLK ");
    //calcClock(USART2, &USART_InitStructure, RCC_ClocksStatus.USART2CLK_Frequency);
    debug("USART3CLK ");
    calcClock(USART3, &USART_InitStructure, RCC_ClocksStatus.USART3CLK_Frequency);
    //debug("CFGR3 & USART3SW ");//RCC->CFGR3 & RCC_CFGR3_USART3SW
    //debug_put_hex16(RCC->CFGR3 & RCC_CFGR3_USART3SW);
    //debug_put_newline();

    debug("PCLK ");
    debug_put_hex32(RCC_ClocksStatus.PCLK_Frequency);
    debug(" SYSCLK ");
    debug_put_hex32(RCC_ClocksStatus.SYSCLK_Frequency);
    debug(" LSE ");
    debug_put_hex32(LSE_VALUE);
    debug(" HSI ");
    debug_put_hex32(HSI_VALUE);
    debug(" HSE ");
    debug_put_hex32(HSE_VALUE);
    debug_put_newline();

    /*
    uint32_t tmp = 0, pllmull = 0, pllsource = 0, prediv1factor = 0;
    tmp = RCC->CFGR & RCC_CFGR_SWS;
    debug("CFGR ");
    debug_put_hex32(tmp);
    pllmull = RCC->CFGR & RCC_CFGR_PLLMULL;
    debug(" pllmull ");
    debug_put_hex32(pllmull);
    pllsource = RCC->CFGR & RCC_CFGR_PLLSRC;
    debug(" pllsource ");
    debug_put_hex32(pllsource);
    pllmull = ( pllmull >> 18) + 2;
    debug(" pllmull ");
    debug_put_hex32(pllmull);
    prediv1factor = (RCC->CFGR2 & RCC_CFGR2_PREDIV1) + 1;
    debug(" prediv1factor ");
    debug_put_hex32(prediv1factor);
    debug(" wat ");
    debug_put_hex32( (HSE_VALUE / prediv1factor) * pllmull);

    debug(" tmp1 ");
    uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
    tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
    debug_put_hex32(tmp);

    debug(" tmp2 ");
    uint8_t APBAHBPrescTable[16] = {0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9};
    tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
    debug_put_hex32(tmp);

    pllmull = RCC->CFGR & RCC_CFGR_PLLMULL;
    pllsource = RCC->CFGR & RCC_CFGR_PLLSRC;
    pllmull = ( pllmull >> 18) + 2;

      prediv1factor = (RCC->CFGR2 & RCC_CFGR2_PREDIV1) + 1;
      // HSE oscillator clock selected as PREDIV1 clock entry
      tmp = (HSE_VALUE / prediv1factor) * pllmull;
    debug(" tmp3 ");
    debug_put_hex32(tmp);

*/
    SystemCoreClockUpdate();

    debug(" SystemCoreClock ");
    debug_put_hex32(SystemCoreClock);

    debug_put_newline();

    USART_Cmd(USART3, ENABLE);

    for (int i = 0; i < 50; ++i) {
        //USART_SendData(USART3, 'W');
        USART_SendData(USART3, (uint16_t) 0x49);
        while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET) {
        }
    }

    //debug("multi4in1: init timer\n");
    multi4in1_init_timer();

    debug("multi4in1: init done\n");
    debug_flush();
}

void multi4in1_init_timer(void) {
    // init timer3 for 9ms
    TIM_TimeBaseInitTypeDef timebase_init;
    TIM_OCInitTypeDef oc_init;
    NVIC_InitTypeDef nvic_init;

    // pre-initialise structs
    TIM_TimeBaseStructInit(&timebase_init);
    TIM_OCStructInit(&oc_init);

    // TIM3 clock enable
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    // Enable the TIM3 gloabal Interrupt
    nvic_init.NVIC_IRQChannel = TIM3_IRQn;
    nvic_init.NVIC_IRQChannelPriority = NVIC_PRIO_FRSKY;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);

    // compute prescaler value
    // we want one ISR every 9ms
    // setting TIM_Period to 9000 will reuqire
    // a prescaler so that one timer tick is 1us (1MHz)
    uint16_t prescaler = (uint16_t)(SystemCoreClock / 1000000) - 1;

    // time base configuration as calculated above
    // timer counts with 1MHz thus 9000 ticks = 9ms
    timebase_init.TIM_Period = 9000 - 1;
    timebase_init.TIM_Prescaler = prescaler;
    timebase_init.TIM_ClockDivision = 0;
    timebase_init.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &timebase_init);

    // should be done by timebaseinit...
    // TIM_PrescalerConfig(TIM3, prescaler, TIM_PSCReloadMode_Immediate);

    // TIM Interrupts enable
    // DO NOT ENABLE IT YET
    // TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    // TIM3 enable counter
    TIM_Cmd(TIM3, ENABLE);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
}

void TIM3_IRQHandler(void) {
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

        //USART_SendData(USART3, 'A');
        counter++;
        counter = counter > 255 ? 0 : counter;
        USART_SendData(USART3, counter);
        // Wait until transmit finishes
        while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET) {

        }

        TIM_SetAutoreload(TIM3, 9000 - 1);
    }
}
