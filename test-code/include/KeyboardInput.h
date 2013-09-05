#include <iostream>
#include <fstream>
#include <termio.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>

class KeyboardInput
{
public:

    KeyboardInput();
    ~KeyboardInput();
    void init();

    int tty_unbuffered(int);     
    int tty_reset(int);
    void tty_atexit(void);

    struct termios save_termios;
    int ttysavefd;
};
