#include "hubo_plus.h"
#include <iostream>

 
int main(int argc, char **argv)
{
    hubo_plus hubo;
    int i=0, imax=20
;
/*    hubo.setJointNominalSpeed( LAR, 0.2 );
    hubo.setJointNominalSpeed( LAP, 0.2 );
    hubo.setJointNominalSpeed( RAR, 0.2 );
    hubo.setJointNominalSpeed( RAP, 0.2 );
*/
    hubo.setJointNominalAcceleration( LAR, 0.6 );
    hubo.setJointNominalAcceleration( LAP, 0.6 );
    hubo.setJointNominalAcceleration( RAP, 0.6 );
    hubo.setJointNominalAcceleration( RAR, 0.6 );

    hubo.setJointNominalAcceleration( RKN, 0.6 );
    hubo.setJointNominalAcceleration( RHP, 0.6 );
    hubo.setJointNominalAcceleration( LKN, 0.6 );
    hubo.setJointNominalAcceleration( LHP, 0.6 );

    

    hubo.setJointAngleMax( RHP, 0 );
    hubo.setJointAngleMax( LHP, 0 );

    struct kneeAngle {
        double left;
        double right;
    };
    double height;
    double angle;
    double leftP=0, leftR=0, rightP=0, rightR=0, ptime, dt, knee, LHPVel, RHPVel, weight,
            imuAngleXZero, LHRVel, RHRVel;
    double compRollGain = 0.001;
    double compPitchGain = 0.001;
    double pitchAngleGain = 0.5*M_PI/180;
    double pitchRotVelGain = 0.1*M_PI/180;
    double rollAngleGain = 0.3*M_PI/180;
    double rollRotVelGain = 0.1*M_PI/180;
    double integralGain = 0; //0.05*M_PI/180;
    double anklePitchVelGain = 1.0;
    double ankleRollVelGain = 0.01;

    double compLP;
    double compLR;
    double compRP;
    double compRR;
    std::string command = "";
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
  
            //Bend down and then stand back up
            if ((atime - initialTime) > 5 && ((atime - initialTime) <= 10) && (hubo.getJointAngle( RKN ) < 1.0))
            {
                knee = 0.15;
                command = "Down ";
            }

            else if ((atime - initialTime) > 12 && hubo.getJointAngle( RKN ) > 0.01)
            {
                knee = -0.15;
                command = "Up   ";
            }

            else if ((atime - initialTime) > 12 && hubo.getJointAngle( RKN ) <= 0.01)
            {
                knee = 0;
                command = "Go Again";
                initialTime = hubo.getTime();
                atime = hubo.getTime();
            }

            else if ((atime - initialTime) <= 20)
            {
                knee = 0;
                command = "Wait ";
            }

            else if ((hubo.getJointAngle( RKN ) > 1.0 || hubo.getJointAngle( LKN ) > 1.0) && (atime - initialTime) <= 10)
            {
                knee = 0;
                command = "limit";
            }
        
            else {
                command = "Doing nothing";
               } 
 
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

//            hubo.setJointVelocity( RHR, RHRVel );
//            hubo.setJointVelocity( LHR, LHRVel );

            hubo.sendControls();
 
            // Display IMU readings
            if( i==imax )
//                std::cout << "Weight:" << hubo.getLeftFootFz() + hubo.getRightFootFz()  <<  "\tLP:" << leftP << "\tRP:" << rightP << "\tLR:" << leftR << "\tRR:" << rightR << std::endl;
                std::cout << "Command: " << command << "\tTime: " << atime-initialTime << "\tWeight:" << hubo.getLeftFootFz() + hubo.getRightFootFz() <<  "\tLKN Angle: " << hubo.getJointAngle(LKN) << "\tRKN Angle: " << hubo.getJointAngle(RKN) << std::endl;

//                std::cout << "IMU:" << "\tAngleX:" << hubo.getAngleX() << "\tAngleY:" << hubo.getAngleY()
//                        << "\tRotVelX:" << hubo.getRotVelX() << "\tRotVelY:" << hubo.getRotVelY()
//                        << std::endl;

/*            // Display feet inclinometer readings
            if( i==imax )
                std::cout << "LAccX:" << hubo.getLeftAccX() << "\tLAccY:" << hubo.getLeftAccY()
                        << "\tLAccZ:" << hubo.getLeftAccZ()-9.8
                        << "\tRAccX:" << hubo.getRightAccX() << "\tRAccY:" << hubo.getRightAccY()
                        << "\tRAccZ:" << hubo.getRightAccZ()-9.8 << std::endl;
*///                std::cout << "RAP:" << rightP << "\t" << "RAR:" << rightR << "\t"
//                    <<   "LAP:" << leftP  << "\t" << "LAR:" << leftR << std::endl;
        }
        
        ptime = hubo.getTime();
    }

/*    int bendLeg(std::string leg, double height)
    {
        angle = acos(height/(2*(l1 + l2)))*2;

        if (angle >= 1.0)
            return -1;
        if(leg == "both" || leg = "Both") {
            kneeAngle.left = kneeAngle.right = angle;
        } else if (leg == "left") {
            kneeAngle.left = angle;
        } else if (leg == "right") {
            kneeAngle.right = angle;
        }
    }*/

}
