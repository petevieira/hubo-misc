#include <Hubo_Control.h>
#include <iostream>

 
int main(int argc, char **argv)
{
    Hubo_Control hubo;
    int i=0, imax=30;
    double ptime, dt;
    ArmVector rightArmAngleStates, leftArmAngleStates;
    LegVector rightLegAngleStates, leftLegAngleStates;
    Eigen::Matrix<double,5,1> rf, lf;

    ptime = hubo.getTime();

    // redirect signals so Ctrl-C causes graceful termination
    redirectSigs();

    // main loop
    while(!daemon_sig_quit)
    {
        hubo.update();

        // get change in time
        dt = hubo.getTime() - ptime;

        // if new data was received...
        if( dt > 0 )
        {
            i++; if(i>imax) i=0;

            // get joint states for arms and legs
            hubo.getRightArmAngleStates(rightArmAngleStates); 
            hubo.getLeftArmAngleStates(leftArmAngleStates);            
            hubo.getRightLegAngleStates(rightLegAngleStates);
            hubo.getLeftLegAngleStates(leftLegAngleStates);            

            // get joint states for left fingers
            rf(0) = hubo.getJointAngleState(RF1);
            rf(1) = hubo.getJointAngleState(RF2);
            rf(2) = hubo.getJointAngleState(RF3);
            rf(3) = hubo.getJointAngleState(RF4);
            rf(4) = hubo.getJointAngleState(RF5);

            // get joint states for left fingers
            lf(0) = hubo.getJointAngleState(LF1);
            lf(1) = hubo.getJointAngleState(LF2);
            lf(2) = hubo.getJointAngleState(LF3);
            lf(3) = hubo.getJointAngleState(LF4);
            lf(4) = hubo.getJointAngleState(LF5);

            if( i==imax )
            {
                std::cout << "====IMU===="
                          << "\nImuAngle(X,Y):  " << hubo.getAngleX() << ", "
                                                  << hubo.getAngleY()
                          << "\nImuRotVel(X,Y): " << hubo.getRotVelX() << ", "
                                                  << hubo.getRotVelY()
                          << "\n\n====F/T Hands===="
                          << "\nRightHandMx: " << hubo.getRightHandMx()
                          << "\nRightHandMy: " << hubo.getRightHandMy()
                          << "\nLeftHandMx:  " << hubo.getLeftHandMx()
                          << "\nLeftHandMy:  " << hubo.getLeftHandMy()
                          << "\n\n====F/T Feet===="
                          << "\nRightFootM(X,Y): " << hubo.getRightFootMx() << ", "
                                                   << hubo.getRightFootMy()
                          << "\nRightFootFz: " << hubo.getRightFootFz()
                          << "\nLeftFootM(X,Y):  " << hubo.getLeftFootMx() << ", "
                                                   << hubo.getLeftFootMy()
                          << "\nLeftFootFz:  " << hubo.getLeftFootFz()
                          << "\n\n====Feet Tilt===="
                          << "\nRightTilt(X,Y,Z): " << hubo.getRightTiltX() << ", "
                                                    << hubo.getRightTiltY() << ", "
                                                    << hubo.getRightTiltZ()
                          << "\nLeftTilt(X,Y,Z):  " << hubo.getLeftTiltX() << ", "
                                                    << hubo.getLeftTiltY() << ", "
                                                    << hubo.getLeftTiltZ()
                          << "\n\n====Arm States===="
                          << "\nRight Arm: " << rightArmAngleStates.transpose()
                          << "\nLeft Arm:  " << leftArmAngleStates.transpose()
                          << "\n\n====Leg States===="
                          << "\nRight Leg: " << rightLegAngleStates.transpose()
                          << "\nLeft Leg:  " << leftLegAngleStates.transpose()
                          << "\n\n====Finger States===="
                          << "\nRight Fingers: " << rf.transpose()
                          << "\nLeft Fingers:  " << lf.transpose()
                          << "\n\n====Aux States===="
                          << "\nWaist:       " << hubo.getJointAngleState(WST)
                          << "\nNeck(Y,1,2): " << hubo.getJointAngleState(NKY) << ", "
                                               << hubo.getJointAngleState(NK1) << ", "
                                               << hubo.getJointAngleState(NK2)
                          << std::endl;
            }
        }
        
        ptime = hubo.getTime();
    }

}
