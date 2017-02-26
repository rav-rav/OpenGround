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


void multi4in1_init(void) {
    debug("multi4in1: init usart\n");
    debug_flush();
    multi4in1_init_usart();

    debug("multi4in1: init timer\n");
    debug_flush();
    multi4in1_init_timer();

    debug("multi4in1: init done\n");
    debug_flush();
}

void multi4in1_init_usart(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    // Turn on USART1
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    // Turn on IO Port C
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

    // Configure USART3 rx tx as push-pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    //enable alternate function 1: usart
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource4, GPIO_AF_1);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource5, GPIO_AF_1);

    //set baudrate
    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.USART_BaudRate = 115200;
    USART_Init(USART3, &USART_InitStructure);

    //enable usart
    USART_Cmd(USART3, ENABLE);
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