#include <Hubo_Tech.h>
#include <iostream>
#include <fstream>
#include "Fastrak.h"
 
int main(int argc, char **argv)
{
//    hubo_plus hubo("balance-test");
    /* Instantiate Hubo_Tech object from constructor */
    Hubo_Tech hubo;
//    std::ofstream myfile;
//    myfile.open("balance-output.txt");
    Fastrak fastrak;

    /* Initialize Fastrak */
    fastrak.initFastrak();

    /* Set Left Leg Joint Nominal Accelerations */
    hubo.setJointNominalAcceleration( LKN, 0.6 );
    hubo.setJointNominalAcceleration( LHP, 0.6 );
    hubo.setJointNominalAcceleration( LHR, 0.6 );
    hubo.setJointNominalAcceleration( LAR, 0.6 );
    hubo.setJointNominalAcceleration( LAP, 0.6 );
 
    /* Set Right Leg Joint Nominal Accelerations */
    hubo.setJointNominalAcceleration( RKN, 0.6 );
    hubo.setJointNominalAcceleration( RHP, 0.6 );
    hubo.setJointNominalAcceleration( RHR, 0.6 );
    hubo.setJointNominalAcceleration( RAP, 0.6 );
    hubo.setJointNominalAcceleration( RAR, 0.6 );

    /* Set max joint angles for both leg hip pitch joints */
    hubo.setJointAngleMax( RHP, 0 );
    hubo.setJointAngleMax( LHP, 0 );

    /* Local Variables */
    double L1 = 2*0.3002;
    double L2 = 0.28947 + 0.0795;
    double height = 0;
    Eigen::Vector3d fastrakOrigin;
    Eigen::Vector3d trans; 
    Eigen::Quaterniond quat;

    double leftP=0, leftR=0, rightP=0, rightR=0; 
    double knee, kneeVel, kneeAngleError;

    double compRollGain = 0.0015;
    double compPitchGain = 0.0015;
    double pitchAngleGain = 0.5*M_PI/180.0;
    double pitchRotVelGain = 0.0*M_PI/180;
    double rollAngleGain = 0.3*M_PI/180;
    double rollRotVelGain = 0.0*M_PI/180;
    double integralGain = 0; //0.05*M_PI/180;
	double kneeVelGain = 0.2;
    double ptime, dt;
    int i=0, imax=40;

    /* Get latest data from the ach channels */
    hubo.update();

    /* Get current time */
    ptime = hubo.getTime();

    /* Read Fastrak sensor to get sensor origin */
    fastrak.getPose(fastrakOrigin, quat, 1, true);

    /* Print out sensor origin location */
//    std::cout << fastrakOrigin.transpose() << std::endl;

    /* Print headers for output data */
//    fprintf(stdout, "Time,msgID,Height,KneeVel,KneeAngle\n"); 
//    while(!daemon_sig_quit)
    while(true)
    {
        /* Get latest data from the ach channels */
        hubo.update();

        dt = hubo.getTime() - ptime;
        ptime = hubo.getTime();
        if( dt > 0 )      
//        if( true )//dt > 0 )
        {
            /* Reset i to 0 when it reaches max value */
            i++; if(i>imax) i=0;

            /* Read Fastrak to get translation and quaternion to sensor */
            fastrak.getPose( trans, quat, 1, true ); //Using sensor 1
            /* Print out location of fastrak sensor */
//            std::cout << trans.transpose() << std::endl;

            /* Make translation relative to initial sensor location */
			trans -= fastrakOrigin;

            /* Set height equal to the Fastrak z-position plus shoulder height of Hubo */
            height = trans(2) + L1 + L2;

            /* Keep height maximum at shoulder height */
            if( height-L2 > L1 )
                height = L1+L2;
            /* Set minimum height to 0.2 meters above waist */
            else if( height-L2 < 0.25 )
                height = L1 + 0.2; // TODO: Check if this is a good value

            /* Calculate knee angle required for calculated height value */
            knee = acos( (height-L2)/L1 )*2;

            /* Set knee angle error to (desired angle - current angle) */
            kneeAngleError = knee - hubo.getJointAngle( RKN );

            /* Set knee velocity to knee angle error times a gain */
            kneeVel = kneeVelGain*kneeAngleError;			

            /* Calculate ankle roll and pitch velocities by resisting IMU angle, complying with
             * the moment about the ankle (each with gains) and set the pitch velocities to 
             * negative half the knee velocities
             */
            leftP = pitchAngleGain*hubo.getAngleY() - compPitchGain*hubo.getLeftFootMy() + (-kneeVel/2);
            leftR = rollAngleGain*hubo.getAngleX() - compRollGain*hubo.getLeftFootMx();
            rightP = pitchAngleGain*hubo.getAngleY() - compPitchGain*hubo.getRightFootMy() + (-kneeVel/2);
            rightR = rollAngleGain*hubo.getAngleX() - compRollGain*hubo.getRightFootMx();

            /* Set joint velocities */
            hubo.setJointVelocity( RAP, rightP );
            hubo.setJointVelocity( RAR, rightR );
            hubo.setJointVelocity( LAP, leftP );
            hubo.setJointVelocity( LAR, leftR );
            
            hubo.setJointVelocity( RKN, kneeVel );
            hubo.setJointVelocity( RHP, -kneeVel/2.0 );
            hubo.setJointVelocity( LKN, kneeVel );
            hubo.setJointVelocity( LHP, -kneeVel/2.0 );

            /* Send commands to the control daemon */
            hubo.sendControls();

//            if( i==imax )
//                fprintf(stdout, "%f,%d,%f,%f\n", ptime, hubo.msgID, height, kneeVel, hubo.getJointAngle(RKN)); 
            if( i==imax )
            {
                std::cout  
                        << "Trans: " << trans.transpose()
//                        << "\nRotatCol1: " << quat.matrix().row(0)
//                        << "\nRotatCol2: " << quat.matrix().row(1)
//                        << "\nRotatCol3: " << quat.matrix().row(2)
                        << std::endl;
                        //<< "\nQuat:" << quat.w() << "\t" << quat.x() << "\t" << quat.y() << "\t" << quat.z()
            }
        }
        
    }
//    myfile.close();
}
