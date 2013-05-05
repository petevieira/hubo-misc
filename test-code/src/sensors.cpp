#include <Hubo_Control.h>
#include <iostream>

 
int main(int argc, char **argv)
{
    Hubo_Control hubo;
    int i=0, imax=30;
    double ptime, dt;
    Vector6d rightArmAngleStates, leftArmAngleStates;
    Vector6d rightLegAngleStates, leftLegAngleStates;
    Eigen::Matrix<double,5,1> rf, lf;

    ptime = hubo.getTime();

    redirectSigs();

    while(!daemon_sig_quit)
    {
        hubo.update();

        dt = hubo.getTime() - ptime;
        
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
                          << "\nImuAngleX:  " << hubo.getAngleX() 
                          << "\nImuAngleY:  " << hubo.getAngleY()
                          << "\nImuRotVelX: " << hubo.getRotVelX()
                          << "\nImuRotVelY: " << hubo.getRotVelY()
                          << "\n\n====F/T Hands===="
                          << "\nRightHandMx: " << hubo.getRightHandMx()
                          << "\nRightHandMy: " << hubo.getRightHandMy()
                          << "\nLeftHandMx:  " << hubo.getLeftHandMx()
                          << "\nLeftHandMy:  " << hubo.getLeftHandMy()
                          << "\n\n====F/T Feet===="
                          << "\nRightFootMx: " << hubo.getRightFootMx()
                          << "\nRightFootMy: " << hubo.getRightFootMy()
                          << "\nRightFootFz: " << hubo.getRightFootFz()
                          << "\nLeftFootMx:  " << hubo.getLeftFootMx()
                          << "\nLeftFootMy:  " << hubo.getLeftFootMy()
                          << "\nLeftFootFz:  " << hubo.getLeftFootFz()
                          << "\n\n====Feet Tilt===="
                          << "\nRightTiltX: " << hubo.getRightTiltX()
                          << "\nRightTiltY: " << hubo.getRightTiltY()
                          << "\nRightTiltZ: " << hubo.getRightTiltZ()
                          << "\nLeftTiltX:  " << hubo.getLeftTiltX()
                          << "\nLeftTiltY:  " << hubo.getLeftTiltY()
                          << "\nLeftTiltZ:  " << hubo.getLeftTiltZ()
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
                          << "\nWaist:    " << hubo.getJointAngleState(WST)
                          << "\nNeck Yaw: " << hubo.getJointAngleState(NKY)
                          << "\nNeck 1:   " << hubo.getJointAngleState(NK1)
                          << "\nNeck 2:   " << hubo.getJointAngleState(NK2)
                          << std::endl;
            }
        }
        
        ptime = hubo.getTime();
    }

}
