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
/*
    hubo.setPositionControl( RKN );
    hubo.setPositionControl( RHP );
    hubo.setPositionControl( LKN );
    hubo.setPositionControl( LHP );

    hubo.setJointNominalSpeed( RKN, 0.05 );
    hubo.setJointNominalSpeed( RHP, 0.025 );
    hubo.setJointNominalSpeed( LKN, 0.05 );
    hubo.setJointNominalSpeed( LHP, 0.025 );
*/

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

    double leftPIntegral = 0.0;
    double leftRIntegral = 0.0;
    double rightPIntegral = 0.0;
    double rightRIntegral = 0.0;
    double pTiltIntegral = 0.0;
    double rTiltIntegral = 0.0;

    double springGainUp = 0.002;
    double springGainDown = 0.0075;

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
  
        /* 
            weight =  hubo.getLeftFootFz() + hubo.getRightFootFz();
            if ( weight > 440) {
                    knee = ( weight - 440 );

                    knee *= springGainDown;

                    if( knee >= 0.3 )
                        knee = 0.3;

                    
            } else {
                if (hubo.getJointAngle( RKN ) + hubo.getJointAngle( RHP ) < .01 && hubo.getJointAngle( LKN) + hubo.getJointAngle( LHP ) < .01) {
                    knee = 0;
                } else {
                knee = ( weight - 440 );

                knee *= springGainUp;

                    if( knee <= -0.3 )
                        knee = -0.3;
                }
            }
          */  
	    knee = 0;
            LHPVel = -knee/2.0;
            RHPVel = -knee/2.0;
 
//            if(atime - initialTime > 10)
//            {
//                imuAngleXZero = 0.1478;
//                command = "lean";
//            }

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
		        std::cout << "Command: " << command << "\tWeight:" << hubo.getLeftFootFz() + hubo.getRightFootFz() << "\tLHP Angle:" << hubo.getJointAngle(LHP) << "\tRHP Angle:" << hubo.getJointAngle(RHP) << "\tHipErr: " << hubo.getJointAngle(LHP)-hubo.getJointAngle(RHP) <<  "\tLKN Angle: " << hubo.getJointAngle(LKN) << "\tRKN Angle: " << hubo.getJointAngle(RKN) << "\tKneeErr: " << hubo.getJointAngle(LKN)-hubo.getJointAngle(RKN) << "\tMx: " << hubo.getMx(HUBO_FT_R_FOOT) << "\tMy: " << hubo.getMy(HUBO_FT_R_FOOT) << "\tIMUx: " << hubo.getAngleX() << "\tIMUy: " << hubo.getAngleY() << std::endl;

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
