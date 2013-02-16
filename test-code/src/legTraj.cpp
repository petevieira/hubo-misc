#include <Hubo_Tech.h>

int main(int argc, char **argv)
{
    Hubo_Tech hubo;

    hubo.setJointAngle(REB, -.5);
    hubo.sendControls();
}
