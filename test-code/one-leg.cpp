#include "Hubo_Tech.h"
#include <iostream>

 
int main(int argc, char **argv)
{
    Hubo_Tech hubo;
    int i=0, imax=20
;
    hubo.setJointNominalAcceleration( LAR, 0.6 );
    hubo.setJointNominalAcceleration( LAP, 0.6 );
    hubo.setJointNominalAcceleration( RAP, 0.6 );
    hubo.setJointNominalAcceleration( RAR, 0.6 );

    hubo.setJointNominalAcceleration( RKN, 0.6 );
    hubo.setJointNominalAcceleration( RHP, 0.6 );
    hubo.setJointNominalAcceleration( LKN, 0.6 );
    hubo.setJointNominalAcceleration( LHP, 0.6 );

    enum side { LEFT, RIGHT };
    struct kneeAngle {
        double left;
        double right;
    };
    double height;
    double angle;
    double leftP=0, leftR=0, rightP=0, rightR=0, ptime, dt, knee, LHPVel, RHPVel, weight,
            imuAngleXZero, LHRVel, RHRVel;
    double compGainAnkleRoll = 0.001;
    double compGainAnklePitch = 0.001;
    double anklePitchVelGain = 0.5*M_PI/180;
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
    std::string leg = "";
    double initialTime = hubo.getTime();
    ptime = hubo.getTime();
    double atime = hubo.getTime();

    while(true)
    {
        hubo.update();  ///get latest data from ach channels

        comVector = hubo.getCOM();  ///get center of mass vector for hubo

        standOnLeg(LEFT);   ///stand on one leg
    }       

void standOnLeg(int side)
{
    std::vector<Eigen::Isometry3d> footTransf(2);   ///1 is left, 2 is right
    std::vector<Eigen::Vector6d> legAngles(2);   ///1 is left, 2 is right

    while(true)
    {
        /* Get necessary information */
        hubo.update();  ///get latest data from ach channels
        hubo.getLegAngles(side, qLeg(side));   ///get Left leg joint angles
        hubo.huboLegFK(footTransf(side), qLeg(side), side); ///get transformation for 'side' foot
        comVector = hubo.getCOM(); ///get center of mass vector for hubo

        /* Compute error between actual COM and desired location of COM */            
        yError = footTransf(side) - comVector(1);   ///compute error between y-pos of COM and center of 'side' foot
        xError = footTransf(side) - comVector(0);   ///compute error between x-pos of COM and x=0

        /* Get moments about the ankle roll and pitch axes */
        leftAnkleMx = hubo.getLeftFootMx();
        leftAnkleMy = hubo.getLeftFootMy();
        rightAnkleMx = hubo.getRightFootMx();
        rightAnkleMy = hubo.getRightFootMy();

        /* Compute ankle joint velocities using resistant and compliant terms */
        velLAP = (anklePitchVelGain * xError) - compGainAnklePitch*leftAnkleMy;
        velLAR = (ankleRollVelGain * yError) - compGainAnkleRoll*leftAnkleMx;
        velRAP = (anklePitchVelGain * xError) - compGainAnklePitch*rightAnkleMy;
        velRAR = (ankleRollVelGain * yError) - compGainAnkleRoll*rightAnkleMx;

        /* Compute hip joint velocities */
        velLHR = -velLAR;    ///set LHR velocity opposite to LAR velocity
        velRHR = -velRAR;    ///set RHR velocity opposite to RAR velocity

        /* Send commands to control-daemon */
        hubo.sendControls();
}















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
 
//            if(atime - initialTime > 10)
//            {
//                imuAngleXZero = 0.1478;
//                command = "lean";
//            }

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
