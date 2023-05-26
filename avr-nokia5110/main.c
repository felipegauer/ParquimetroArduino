/* Nokia 5110 LCD AVR Library example
 *
 * Copyright (C) 2015 Sergey Denisov.
 * Written by Sergey Denisov aka LittleBuster (DenisovS21@gmail.com)
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public Licence
 * as published by the Free Software Foundation; either version 3
 * of the Licence, or (at your option) any later version.
 *
 * Original library written by SkewPL, http://skew.tk
 * Custom char code by Marcelo Cohen - 2021
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>

#include "nokia5110.h"

volatile uint8_t timerOverflow = 0;
volatile double money = 0;
volatile uint8_t red = 0;

uint8_t glyph[] = {0b00010000,
                   0b00100100,
                   0b11100000,
                   0b00100100,
                   0b00010000};

void timer1_init()
{
    // Configura o Timer1 para operar em modo CTC
    TCCR1B |= (1 << WGM12);

    // Configura a fonte de clock do Timer1 com o prescaler de 256
    TCCR1B |= (1 << CS12);
    OCR1A = 62500; // (16 MHz / (256 * 1 Hz))

    TIMSK1 |= (1 << OCIE1A);

    /*
        // inicializa timer no modo 7, fast PWM
        // seta OC2B para bottom, limpa OC2B em compare match
        TCCR2A = (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);

        // prescaler é 256
        TCCR2B = (1 << CS22) | (1 << CS21) | (1 << WGM22);

        OCR2A = 100;
        OCR2B = 100;  //duty cycle

    */

    // Habilita as interrupções globais
    sei();
}

int main(void)
{
    DDRC |= (1 << PC5); // verde
    DDRC |= (1 << PC4); // amarelo
    DDRD |= (1 << PD3); // vermelho

    DDRB &= ~(1 << PB0); // 0.25
    DDRD &= ~(1 << PD6); // 0.50
    DDRD &= ~(1 << PD7); // 1.00
    DDRD &= ~(1 << PD5); // Start

    PORTC ^= (1 << PC5);

    char msg[30];
    char moneydouble[4];
    cli();
    // Inicializa o Timer1
    timer1_init();

    nokia_lcd_init();
    nokia_lcd_clear();
    nokia_lcd_custom(1, glyph);
    while (1)
    {
        if (PINB & (1 << PB0))
        {
            while (PINB & (1 << PB0))
            {
                _delay_ms(1); // debounce
            }
            if (money >= 1.50 || money + .25 > 1.50)
            {
                money = 1.50;
            }
            else
            {
                money += .25;
            }
        }

        if (PIND & (1 << PD7))
        {
            while (PIND & (1 << PD7))
            {
                _delay_ms(1); // debounce
            }
            if (money >= 1.50 || money + .50 > 1.50)
            {
                money = 1.50;
            }
            else
            {
                money += .50;
            }
        }

        if (PIND & (1 << PD6))
        {
            while (PIND & (1 << PD6))
            {
                _delay_ms(1); // debounce
            }
            if (money >= 1.50 || money + 1.00 > 1.50)
            {
                money = 1.50;
            }
            else
            {
                money += 1.00;
            }
        }
        nokia_lcd_clear();

        dtostrf(money, 3, 2, moneydouble);
        sprintf(msg, "Dinheiro: %s", moneydouble);
        nokia_lcd_write_string(msg, 1);
        nokia_lcd_render();

        if (money >= 1.50 && PIND & (1 << PD5)) // se tiver dinheiro suficiente e pino PD5 pressionado
        {
            money = 0;
            TCNT1 = 0;
            timerOverflow = 10;   // contagem regressiva em 10
            PORTC &= ~(1 << PC5); // apaga Luz verde
            PORTC |= (1 << PC4);  // Acende luz amarela
            while (1)
            {

                nokia_lcd_clear();
                dtostrf(money, 3, 2, moneydouble);
                sprintf(msg, "Dinheiro: %sTempo: %d", moneydouble, timerOverflow);
                nokia_lcd_write_string(msg, 1);
                nokia_lcd_render();
                if (timerOverflow <= 0)
                {
                    red = 1;
                    timerOverflow = 10;
                    TCNT1 = 0;
                    nokia_lcd_clear();
                    dtostrf(money, 3, 2, moneydouble);
                    sprintf(msg, "Dinheiro: %sTempo ACABOU!", moneydouble);
                    nokia_lcd_write_string(msg, 1);
                    nokia_lcd_render();
                    PORTC &= ~(1 << PC4);

                    while (1)
                    {
                        if (PINB & (1 << PB0) || PIND & (1 << PD7) || PIND & (1 << PD6))
                        {
                            timerOverflow = 0;
                            TCNT1 = 0;
                            red = 0;
                            PORTD &= ~(1 << PD3);
                            PORTC |= (1 << PC5);

                            break;
                        }
                    }
                    break;
                }
            }
        }
    }
}

ISR(TIMER1_COMPA_vect)
{
    TCNT1 = 0;
    timerOverflow--;
    if (red)
    {
        PORTD ^= (1 << PD3);
    }
}
