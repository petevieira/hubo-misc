/* on Hubo first open the fastrak ach channel
   ach -C fastrak			    // this creates the fastrak ach channel on hubo
*/
/* fastrak must be started on local computer first.
   ./fastrakd                       	    // this starts the fastrak daemon
   ach -C fastrak		            // this creates the fastrak ach channel
   achd -r push 192.168.245 fastrak-chan    // this starts sending fastrak data to hubo
*/

#include <Hubo_Control.h>
#include <iostream>
#include "Teleop.h"
 
int main(int argc, char **argv)
{
    Hubo_Control hubo;
    Teleop teleop;
    int i=0, imax=40;

    hubo.setJointNominalAcceleration( LAR, 0.6 );
    hubo.setJointNominalAcceleration( LAP, 0.6 );
    hubo.setJointNominalAcceleration( RAP, 0.6 );
    hubo.setJointNominalAcceleration( RAR, 0.6 );

    hubo.setJointNominalAcceleration( RKN, 0.3 );
    hubo.setJointNominalAcceleration( RHP, 0.3 );
    hubo.setJointNominalAcceleration( LKN, 0.3 );
    hubo.setJointNominalAcceleration( LHP, 0.3 );

    hubo.setJointNominalAcceleration( RHR, 0.6 );
    hubo.setJointNominalAcceleration( LHR, 0.6 );
    

    hubo.setJointAngleMax( RHP, 0 );
    hubo.setJointAngleMax( LHP, 0 );

    double L1 = 2*0.3002;
    double L2 = 0.28947 + 0.0795;
    double height = 0;
    Eigen::Vector3d trans; Eigen::Quaterniond quat;

    double leftP=0, leftR=0, rightP=0, rightR=0, ptime, dt, knee, kneeVel, kneeAngleError;

    double compRollGain = 0.001;
    double compPitchGain = 0.001;
    double pitchAngleGain = 0.5*M_PI/180;
    double pitchRotVelGain = 0.0*M_PI/180;
    double rollAngleGain = 0.3*M_PI/180;
    double rollRotVelGain = 0.0*M_PI/180;
    double integralGain = 0; //0.05*M_PI/180;
    double kneeVelGain = 0.2;
    double leftPIntegral = 0.0;
    double leftRIntegral = 0.0;
    double rightPIntegral = 0.0;
    double rightRIntegral = 0.0;
    double pTiltIntegral = 0.0;
    double rTiltIntegral = 0.0;

    double springGainUp = 0.001;
    double springGainDown = 0.0075;

    double compLP;
    double compLR;
    double compRP;
    double compRR;

    ptime = hubo.getTime();
    double atime = hubo.getTime();
	Eigen::Vector3d fastrakOrigin;
	hubo.getFastrak(fastrakOrigin, quat);

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
  
            hubo.getFastrak( trans, quat );
            if( i==imax )
                std::cout << trans.transpose() << "\t";
			trans -= fastrakOrigin;	
            height = trans(2) + L1 + L2;
            if( height-L2 > L1 )
                height = L1+L2;
            else if( height-L2 < 0.25 )
                height = L1 + 0.2; // TODO: Check if this is a good value

            knee = acos( (height-L2)/L1 )*2;
			kneeAngleError = knee - hubo.getJointAngle( RKN );
		    	
			kneeVel = kneeVelGain*kneeAngleError;			

            leftP = pitchAngleGain*hubo.getAngleY() - compPitchGain*compLP + (-kneeVel/2);
            leftR = rollAngleGain*hubo.getAngleX() - compRollGain*compLR;
            rightP = pitchAngleGain*hubo.getAngleY() - compPitchGain*compRP + (-kneeVel/2);
            rightR = rollAngleGain*hubo.getAngleX() - compRollGain*compRR;

            hubo.setJointVelocity( RAP, rightP );
            hubo.setJointVelocity( RAR, rightR );
            hubo.setJointVelocity( LAP, leftP );
            hubo.setJointVelocity( LAR, leftR );
            
            hubo.setJointVelocity( RKN, kneeVel );
            hubo.setJointVelocity( RHP, -kneeVel/2.0 );
            hubo.setJointVelocity( LKN, kneeVel );
            hubo.setJointVelocity( LHP, -kneeVel/2.0 );
            
            hubo.sendControls();
 
            // Display IMU readings
            if( i==imax )
                std::cout << "Height:" << height << "\tkneeAngle: " << knee << "\tkneeVel: " << kneeVel << std::endl;
        }
        
        ptime = hubo.getTime();
    }
}
