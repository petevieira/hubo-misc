#include <Hubo_Control.h>
#include <iostream>

 
int main(int argc, char **argv)
{
    Hubo_Control hubo;
    int i=0, imax=40;

    double ptime, dt;
    double initialTime = hubo.getTime();
    ptime = hubo.getTime();
    while(!daemon_sig_quit)
    {
        hubo.update();

        dt = hubo.getTime() - ptime;
        
        if( dt > 0 )
        {
            i++; if(i>imax) i=0;

            if( i==imax )
            {
                std::cout << "WST: " << hubo.getJointAngle(REB) 
                          << "\tRF1: " << hubo.getJointAngle(RF1)
                          << std::endl;
            }
        }
        
        ptime = hubo.getTime();
    }

}
