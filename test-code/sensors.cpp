#include "hubo_plus.h"
#include <iostream>

 
int main(int argc, char **argv)
{
    hubo_plus hubo;
    int i=0, imax=20;

    double ptime, dt;
    double initialTime = hubo.getTime();
    ptime = hubo.getTime();

    while(true)
    {
        hubo.update();

        dt = hubo.getTime() - ptime;
        
        if( dt > 0 )
        {
            i++; if(i>imax) i=0;

            if( i==imax )
//                std::cout << "Weight:" << hubo.getLeftFootFz() + hubo.getRightFootFz()  <<  "\tLP:" << leftP << "\tRP:" << rightP << "\tLR:" << leftR << "\tRR:" << rightR << std::endl;
                std::cout << "\tWeight:" << hubo.getLeftFootFz() + hubo.getRightFootFz() << "\tLHP Angle:" << hubo.getJointAngle(LHP) << "\tRHP Angle:" << hubo.getJointAngle(RHP) << "\tHipErr: " << hubo.getJointAngle(LHP)-hubo.getJointAngle(RHP) <<  "\tLKN Angle: " << hubo.getJointAngle(LKN) << "\tRKN Angle: " << hubo.getJointAngle(RKN) << "\tKneeErr: " << hubo.getJointAngle(LKN)-hubo.getJointAngle(RKN) << "\tMx: " << hubo.getMx(HUBO_FT_R_FOOT) << "\tMy: " << hubo.getMy(HUBO_FT_R_FOOT) << "\tIMUx: " << hubo.getAngleX() << "\tIMUy: " << hubo.getAngleY() << std::endl;

        }
        
        ptime = hubo.getTime();
    }

}
