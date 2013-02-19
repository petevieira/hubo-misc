#include <Hubo_Tech.h>

int main(int argc, char **argv)
{
    Hubo_Tech hubo;

    hubo.setJointAngle(REB, -.5);
    hubo.sendControls();

    double i, imax=20;

    while(true)
    {
        i++;
        if(i = imax)
        {
            hubo.update();
            std::cout << "REB-State: " << hubo.getJointAngleState(REB)
                      << "REF-Refer: " << hubo.getJointAngle(REB)
                      << std::endl;
            i = 0;
        }
    }
}
