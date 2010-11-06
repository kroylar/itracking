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

//#define DEBUG

uint8_t ch;

static void ioinit(void) {
    // OC1A (X) and OC1B (Y) as output
    DDRB = 0x06;

    // fast PWM clk/8 
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
    TCCR1B = (1 << CS11) | (1 << WGM13) | (1 << WGM12);
    OCR1A = 1300; // start at 1.5ms
    OCR1B = 1300; // start at 1.5ms

    ICR1 = 20000; // 20ms

    uart_init();
}

FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

int main(void) {
    char strbuf[80];
    int newval;

    ioinit();

    stdout = stdin = stderr = &uart_str;

#ifdef DEBUG
    fprintf(stderr, "Enter a new value for OCR1A and OCR1B to control the pulsewidth.\n");
#endif

    while(1) {
#ifdef DEBUG
        fprintf(stdout, "OCR1A = ?\n");
#endif
        if (fgets(strbuf, sizeof strbuf - 1, stdin) == NULL) break;
        newval = atoi(strbuf);
        if (newval < 0 || newval > 19999) {
#ifdef DEBUG
            fprintf(stdout, "Please enter a value between 0 and 19999.\n");
#endif
        } else {
            OCR1A = newval;
#ifdef DEBUG
            fprintf(stdout, "wrote 0x%x to OCR1A\n", newval);
#endif
        }
#ifdef DEBUG
        fprintf(stdout, "OCR1B = ?\n");
#endif
        if (fgets(strbuf, sizeof strbuf - 1, stdin) == NULL) break;
        newval = atoi(strbuf);
        if (newval < 0 || newval > 19999) {
#ifdef DEBUG
            fprintf(stdout, "Please enter a value between 0 and 19999.\n");
#endif
        } else {
            OCR1B = newval;
#ifdef DEBUG
            fprintf(stdout, "wrote 0x%x to OCR1B\n", newval);
#endif
        }
    }

    return 0;
}

