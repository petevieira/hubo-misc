#include <Hubo_Tech.h>
#include <iostream>

 
int main(int argc, char **argv)
{
    Hubo_Tech hubo;


/* REQUIREMENTS
    Monitor body COM and compensate
    
    Monitor external forces and compensate (FUTURE)

    int i=0, imax=20;
    double compLP;
    double compLR;
    double compRP;
    double compRR;
    std::string command = "";
    std::string leg = "";
    double initialTime = hubo.getTime();
    ptime = hubo.getTime();
    double atime = hubo.getTime();

    while(true)
    {
        hubo.update();

        dt = hubo.getTime() - ptime;
        atime += dt;
        
        if( dt > 0 )
        {
            i++; if(i>imax) i=0;

            compLP = hubo.getLeftFootMy();
            compLR = hubo.getLeftFootMx();
            compRP = hubo.getRightFootMy();
            compRR = hubo.getRightFootMx();
  
	    knee = 0;
            LHPVel = -knee/2.0;
            RHPVel = -knee/2.0;

            hubo.setJointVelocity( RKN, knee );
            hubo.setJointVelocity( RHP, RHPVel );
            hubo.setJointVelocity( LKN, knee );
            hubo.setJointVelocity( LHP, LHPVel );
            
            leftP = pitchAngleGain*hubo.getAngleY() - compPitchGain*compLP + anklePitchVelGain*LHPVel;
            leftR = rollAngleGain*(hubo.getAngleX() - imuAngleXZero) - compRollGain*compLR;
            rightP = pitchAngleGain*hubo.getAngleY() - compPitchGain*compRP + anklePitchVelGain*RHPVel;
            rightR = rollAngleGain*(hubo.getAngleX() - imuAngleXZero) - compRollGain*compRR;

            hubo.setJointVelocity( RAP, rightP );
            hubo.setJointVelocity( RAR, rightR );
            hubo.setJointVelocity( LAP, leftP );
            hubo.setJointVelocity( LAR, leftR );

            hubo.sendControls();
 
            if( i==imax )
        }
        
        ptime = hubo.getTime();
    }
}
