/*!
    \file    main.c
    \brief   USART DMA transmitter receiver

    \version 2017-02-10, V1.0.0, firmware for GD32F30x
    \version 2018-10-10, V1.1.0, firmware for GD32F30x
    \version 2018-12-25, V2.0.0, firmware for GD32F30x
    \version 2020-09-30, V2.1.0, firmware for GD32F30x
*/

/*
    Copyright (c) 2020, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32f30x.h"
#include <stdio.h>
#include "gd32f30x_usart.h"
#include "gd32f30x_gpio.h"
#include <string.h>

char SEND[20];
char GET[20];
uint8_t A;

char rx_buf[64];
uint8_t rx_idx = 0;
volatile uint8_t rx_ready = 0;

#define TIMER TIMER2
void timer_delay_init(void);
void timer_delay_us(uint32_t us);
void timer_delay_ms(uint32_t ms);

int main(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_USART1);
    
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_3);
    
    usart_deinit(USART1);
    usart_baudrate_set(USART1, 115200);
    usart_word_length_set(USART1, USART_WL_8BIT);
    usart_stop_bit_set(USART1, USART_STB_1BIT);
    usart_parity_config(USART1, USART_PM_NONE);
    usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);
    usart_receive_config(USART1, USART_RECEIVE_ENABLE);
    usart_enable(USART1);
    nvic_irq_enable(USART1_IRQn, 0, 0);
    usart_interrupt_enable(USART1, USART_INT_RBNE);

    timer_delay_init();
    
    //printf(SEND,"number=%d", A);
    while(1){
        A = A + 1;
        sprintf(SEND, "number=%d\n", A);
        for(int i = 0; i < strlen(SEND); i++) {
            usart_data_transmit(USART1, (uint8_t)SEND[i]);
            while(RESET == usart_flag_get(USART1, USART_FLAG_TBE));
        }
        timer_delay_ms(500);

        // Эхо строки из буфера
        if (rx_ready) {
            for (uint8_t i = 0; i < strlen((char*)rx_buf); i++) {
                usart_data_transmit(USART1, rx_buf[i]);
                while(RESET == usart_flag_get(USART1, USART_FLAG_TBE));
            }
            usart_data_transmit(USART1, '\n');
            while(RESET == usart_flag_get(USART1, USART_FLAG_TBE));
            rx_ready = 0;
        }
    }
}

void timer_delay_init(void)
{
    rcu_periph_clock_enable(RCU_TIMER2);

    /* Настройка параметров таймера */
    timer_deinit(TIMER);
    timer_parameter_struct timer_initpara;
    timer_initpara.prescaler         = (SystemCoreClock / 1000000) - 1;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 0xFFFFFFFF;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER, &timer_initpara);

    /* Запускаем таймер */
    timer_enable(TIMER);
}

void timer_delay_us(uint32_t us)
{
    uint32_t start = TIMER_CNT(TIMER);
    while ((TIMER_CNT(TIMER) - start) < us);
}

void timer_delay_ms(uint32_t ms)
{
    while (ms--) {
        timer_delay_us(1000);
    }
}

void USART1_IRQHandler(void)
{
    if (usart_interrupt_flag_get(USART1, USART_INT_FLAG_RBNE) != RESET) {
        uint8_t ch = usart_data_receive(USART1);
        if ((ch == '\r' || ch == '\n') && rx_idx > 0) {
            rx_buf[rx_idx] = 0;
            rx_ready = 1; // Флаг: строка готова
            rx_idx = 0;
        } else if (rx_idx < sizeof(rx_buf) - 1 && ch != '\r' && ch != '\n') {
            rx_buf[rx_idx++] = ch;
        }
    }
}

//extern "C" int _write(int file, char* ptr, int len)
//{
//    for(int i = 0; i < len; i++) {
//        usart_data_transmit(USART1, (uint8_t)ptr[i]);
//        while(RESET == usart_flag_get(USART1, USART_FLAG_TBE));
//    }
//    return len;
//}
//extern "C" int fputc(int ch, FILE *f)
//{
//    usart_data_transmit(USART1, (uint8_t)ch);
//    while (RESET == usart_flag_get(USART1, USART_FLAG_TBE));
//    return ch;
//}
