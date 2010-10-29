/***********************************************************************
 * avrpwm
 * 
 * Will accept integer numbers over uart and use them to update the
 * pulse width of the PWM.  Will be used to control the servos.
 *
 * -KRL
 **********************************************************************/

#include <ctype.h>
#include <stdint.h>
#include <stdio.h>

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "uart.h"

uint8_t ch;

static void ioinit(void) {
    // OC1A as output
    DDRB = 0x02;

    // fast PWM clk/8 
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << CS11) | (1 << WGM13) | (1 << WGM12);
    OCR1A = 1499; // start at 1.5ms
    ICR1 = 19999; // 20ms

    uart_init();
}

FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

int main(void) {
    char strbuf[80];
    int newval;

    ioinit();

    stdout = stdin = stderr = &uart_str;

    fprintf(stderr, "Enter a new value for OCR1A to control the pulsewidth.\n");

    while(1) {
        if (fgets(strbuf, sizeof strbuf - 1, stdin) == NULL) break;
        newval = atoi(strbuf);
        if (newval < 0 || newval > 19999) {
            fprintf(stdout, "Please enter a value between 0 and 19999.\n");
        } else {
            OCR1A = newval;
            fprintf(stdout, "wrote 0x%x to OCR1A\n", newval);
        }
    }

    return 0;
}

