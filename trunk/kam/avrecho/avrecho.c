/***********************************************************************
 * avrecho
 * 
 * A simple echo program to test the uart library. Will take user input
 * of up to 80 characters and then spit it right back out.
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
      uart_init();
}

FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

int main(void) {
    char strbuf[80];

    ioinit();

    stdout = stdin = stderr = &uart_str;

    fprintf(stderr, "Hello world!\nType a word to see it echoed!\n");

    while(1) {
        if (fgets(strbuf, sizeof strbuf - 1, stdin) == NULL) break;
        fprintf(stdout, "%s", strbuf);
    }

    return 0;
}

