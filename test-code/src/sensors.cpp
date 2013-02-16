#include <Hubo_Tech.h>
#include <hubo.h>
#include <iostream>

 
int main(int argc, char **argv)
{
    Hubo_Tech hubo;
    int i=0, imax=20;

    ach_channel_t chan_hubo_state;
    struct hubo_state H_state;
    memset( &H_state, 0, sizeof(H_state) );

    // open hubo state
    int r = ach_open(&chan_hubo_state, HUBO_CHAN_STATE_NAME, NULL);

    double ptime, dt;
    double initialTime = hubo.getTime();
    ptime = hubo.getTime();
    size_t fs;
    while(true)
    {
        hubo.update();

        dt = hubo.getTime() - ptime;
        
        if( dt > 0 )
        {
            i++; if(i>imax) i=0;

            r = ach_get( &chan_hubo_state, &H_state, sizeof(H_state), &fs, NULL, ACH_O_LAST );

            if( i==imax )
            {
                std::cout << "REB Angle Ref (rad): " << hubo.getJointAngle(REB) 
                          << "\tREB Angle State (rad): " << H_state.joint[REB].pos
                          << std::endl;
            }
        }
        
        ptime = hubo.getTime();
    }

}
