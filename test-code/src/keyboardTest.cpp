#include <KeyboardInput.h>

int main(int argc, char** argv)
{

    KeyboardInput keyboard;

    char c;

    while(1)
    {
        if ( read(STDIN_FILENO, &c, 1) == 1)
            std::cout << "Pressed " << c << std::endl;
    }

    return 0;
}
