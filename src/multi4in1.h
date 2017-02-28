/*
    Copyright 2016 rav <AT> raav.info

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

    author: rav <AT> raav.info
*/

#ifndef MULTI4IN1_H_
#define MULTI4IN1_H_

#include <stdint.h>

#include "main.h"

void multi4in1_init(void);
void multi4in1_init_usart(void);
void multi4in1_init_timer(void);
void multi4in1_send_packet(void);
void multi4in1_enable_bind(void);

void TIM3_IRQHandler(void);

#endif  // MULTI4IN1_H_
