#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <curses.h>

/* baudrate settings are defined in <asm/termbits.h>, which is
included by <termios.h> */
#define BAUDRATE B9600            
/* change this definition for the correct port */
#define MODEMDEVICE "/dev/ttyUSB5"
#define _POSIX_SOURCE 1 /* POSIX compliant source */

#define FALSE 0
#define TRUE 1

volatile int STOP=FALSE; 
struct termios oldtio;
int fd;

void ex_program(int sig);

main()
{
  (void) signal(SIGINT, ex_program);
  char tmp[10];
  int c, res;
  struct termios newtio;
  char buf[255];
  int OCR1A = 1300, OCR1B = 1300;
  int pwmymin, pwmxmin, pwmymax, pwmxmax;
  
  WINDOW *w = initscr();
  cbreak();
  nodelay(w, TRUE);
/* 
  Open modem device for reading and writing and not as controlling tty
  because we don't want to get killed if linenoise sends CTRL-C.
*/
 fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY ); 
 if (fd <0) {perror(MODEMDEVICE); exit(-1); }

 tcgetattr(fd,&oldtio); /* save current serial port settings */
 memset(&newtio, 0, sizeof(newtio)); /* clear struct for new port settings */

/* 
  BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
  CRTSCTS : output hardware flow control (only used if the cable has
            all necessary lines. See sect. 7 of Serial-HOWTO)
  CS8     : 8n1 (8bit,no parity,1 stopbit)
  CLOCAL  : local connection, no modem contol
  CREAD   : enable receiving characters
*/
 newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | HUPCL | CREAD;
 
/*
  IGNPAR  : ignore bytes with parity errors
  ICRNL   : map CR to NL (otherwise a CR input on the other computer
            will not terminate input)
  otherwise make device raw (no other input processing)
*/
 newtio.c_iflag = IGNBRK;
 
/*
 Raw output.
*/
 newtio.c_oflag = 0;
 
/*
  ICANON  : enable canonical input
  disable all echo functionality, and don't send signals to calling program
*/
 newtio.c_lflag = 0;
 
/* 
  initialize all control characters 
  default values can be found in /usr/include/termios.h, and are given
  in the comments, but we don't need them here
*/
 newtio.c_cc[VINTR]    = 0;     /* Ctrl-c */ 
 newtio.c_cc[VQUIT]    = 0;     /* Ctrl-\ */
 newtio.c_cc[VERASE]   = 0;     /* del */
 newtio.c_cc[VKILL]    = 0;     /* @ */
 newtio.c_cc[VEOF]     = 4;     /* Ctrl-d */
 newtio.c_cc[VTIME]    = 5;     /* inter-character timer unused */
 newtio.c_cc[VMIN]     = 1;     /* blocking read until 1 character arrives */
 newtio.c_cc[VSWTC]    = 0;     /* '\0' */
 newtio.c_cc[VSTART]   = 0;     /* Ctrl-q */ 
 newtio.c_cc[VSTOP]    = 0;     /* Ctrl-s */
 newtio.c_cc[VSUSP]    = 0;     /* Ctrl-z */
 newtio.c_cc[VEOL]     = 0;     /* '\0' */
 newtio.c_cc[VREPRINT] = 0;     /* Ctrl-r */
 newtio.c_cc[VDISCARD] = 0;     /* Ctrl-u */
 newtio.c_cc[VWERASE]  = 0;     /* Ctrl-w */
 newtio.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
 newtio.c_cc[VEOL2]    = 0;     /* '\0' */
 
 newtio.c_ispeed = 0xd;
 newtio.c_ospeed = 0xd;

/* 
  now clean the modem line and activate the settings for the port
*/
 tcflush(fd, TCIFLUSH);
 tcsetattr(fd,TCSANOW,&newtio);

/*
  terminal settings done, now handle input
  In this example, inputting a 'z' at the beginning of a line will 
  exit the program.
*/
#define DELAY 50000

    sleep(1);
    while (getch() == ERR) {
        OCR1B += 1;
        sprintf(tmp,"%d\n",OCR1A);
        write(fd,tmp,5);
        usleep(DELAY);
        sprintf(tmp,"%d\n",OCR1B);
        write(fd,tmp,5);
        usleep(DELAY);
    }
    pwmymax = OCR1B;

    while (getch() == ERR) {
        OCR1A -= 1;
        sprintf(tmp,"%d\n",OCR1A);
        write(fd,tmp,5);
        usleep(DELAY);
        sprintf(tmp,"%d\n",OCR1B);
        write(fd,tmp,5);
        usleep(DELAY);
    }
    pwmxmin = OCR1A;

    while (getch() == ERR) {
        OCR1B -= 1;
        sprintf(tmp,"%d\n",OCR1A);
        write(fd,tmp,5);
        usleep(DELAY);
        sprintf(tmp,"%d\n",OCR1B);
        write(fd,tmp,5);
        usleep(DELAY);
    }
    pwmymin = OCR1B;

    while (getch() == ERR) {
        OCR1A += 1;
        sprintf(tmp,"%d\n",OCR1A);
        write(fd,tmp,5);
        usleep(DELAY);
        sprintf(tmp,"%d\n",OCR1B);
        write(fd,tmp,5);
        usleep(DELAY);
    }
    pwmxmax = OCR1A;
 
 /* restore the old port settings */
 tcsetattr(fd,TCSANOW,&oldtio);
}


void ex_program(int sig) {
    tcsetattr(fd,TCSANOW,&oldtio);
    endwin();
    exit(0);
}
