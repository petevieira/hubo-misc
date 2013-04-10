#include <Hubo_Control.h>
#include <iostream>
#include <fstream>
#include <getopt.h>
#include <termio.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>


static int tty_unbuffered(int);     
static void tty_atexit(void);
static int tty_reset(int);
static void tweak_init();

static struct termios save_termios;
static int  ttysavefd = -1;


int main(int argc, char **argv)
{
    //=== OBJECTS ===//
    // Create Hubo_Control object
    Hubo_Control hubo;
//    std::cout << "Daemonizing as impedanceCtrl\n";
//    Hubo_Control hubo("impedanceCtrl");

    //=== LOCAL VARIABLES ===//
    int i=0, imax=40;
    double dt, ptime;
    double rHandCurrent=0;
    double handCurrentStep=0.25;
    bool print=true;
 
    ptime = hubo.getTime(); // set initial time for dt(0)

    std::cout << "Executing control loop ...\n";

    tweak_init();

    char c;

    while(!daemon_sig_quit)
    {
        // get latest state info for Hubo
        hubo.update();

        dt = hubo.getTime() - ptime;
        ptime = hubo.getTime();

        // if new data is available...
        if(dt > 0)
        {
            if ( read(STDIN_FILENO, &c, 1) == 1) {
                switch (c) {
                    case 's':
                        rHandCurrent += handCurrentStep;
                        if(rHandCurrent > 1.0) rHandCurrent = 1.0;
                        break;
                    case 'x':
                        rHandCurrent -= handCurrentStep;
                        if (rHandCurrent < -1.0) rHandCurrent = -1.0;
                        break;
                    case 'a':
                        lHandCurrent += handCurrentStep;
                        if(lHandCurrent > 1.0) lHandCurrent = 1.0;
                        break;
                    case 'z':
                        lHandCurrent -= handCurrentStep;
                        if (lHandCurrent < -1.0) lHandCurrent = -1.0;
                        break;

                 }
            }

            hubo.passJointAngle(RF1, rHandCurrent);
            hubo.passJointAngle(RF2, rHandCurrent);
            hubo.passJointAngle(RF3, rHandCurrent);
            hubo.passJointAngle(RF4, rHandCurrent);
            hubo.passJointAngle(RF5, rHandCurrent);

            hubo.passJointAngle(LF1, lHandCurrent);
            hubo.passJointAngle(LF2, lHandCurrent);
            hubo.passJointAngle(LF3, lHandCurrent);
            hubo.passJointAngle(LF4, lHandCurrent);
            hubo.passJointAngle(LF5, lHandCurrent);

            hubo.sendControls();

            // print output every imax cycles
            if( i>=imax && print==true )
            {
                std::cout //<< "\033[2J"
                          << "HandCurrent: " << rHandCurrent
                          << std::endl;
            }
            if(i>=imax) i=0; i++;
        }
    }
    return 0;
}


static int
tty_unbuffered(int fd)      /* put terminal into a raw mode */
{
    struct termios  buf;

    if (tcgetattr(fd, &buf) < 0)
        return(-1);
        
    save_termios = buf; /* structure copy */

    /* echo off, canonical mode off */
    buf.c_lflag &= ~(ECHO | ICANON);

    /* 1 byte at a time, no timer */
    buf.c_cc[VMIN] = 1;
    buf.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSAFLUSH, &buf) < 0)
        return(-1);

    ttysavefd = fd;
    return(0);
}

static int
tty_reset(int fd)       /* restore terminal's mode */
{
    if (tcsetattr(fd, TCSAFLUSH, &save_termios) < 0)
        return(-1);
    return(0);
}

static void
tty_atexit(void)        /* can be set up by atexit(tty_atexit) */
{
    if (ttysavefd >= 0)
        tty_reset(ttysavefd);
}

static void
tweak_init()
{
   /* make stdin unbuffered */
    if (tty_unbuffered(STDIN_FILENO) < 0) {
        std::cout << "Set tty unbuffered error" << std::endl;
        exit(1);
    }

    atexit(tty_atexit);

    /* nonblock I/O */
    int flags;
    if ( (flags = fcntl(STDIN_FILENO, F_GETFL, 0)) == 1) {
        perror("fcntl get flag error");
        exit(1);
    }
    if (fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK) == -1) {
        perror("fcntl set flag error");
        exit(1);
    }
}

