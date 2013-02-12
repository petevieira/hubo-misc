#include <Hubo_Tech.h>
#include <iostream>

 
int main(int argc, char **argv)
{
    /* Instantiate Hubo_Tech object from constructor */
    Hubo_Tech hubo;

    /* Set Left Leg Joint Nominal Accelerations */
    hubo.setJointNominalAcceleration( LKN, 0.6 );
    hubo.setJointNominalAcceleration( LHP, 0.6 );
    hubo.setJointNominalAcceleration( LAR, 0.6 );
    hubo.setJointNominalAcceleration( LAP, 0.6 );

    /* Set Right Leg Joint Nominal Accelerations */
    hubo.setJointNominalAcceleration( RKN, 0.6 );
    hubo.setJointNominalAcceleration( RHP, 0.6 );
    hubo.setJointNominalAcceleration( RAP, 0.6 );
    hubo.setJointNominalAcceleration( RAR, 0.6 );

    /* Local Variables */
    Vector6d currentLeftLegAngles, currentRightLegAngles;
    Vector6d refLeftLegAngles, refRightLegAngles;
    Vector6d leftLegError, rightLegError;
    Eigen::Isometry3d leftFootTransform, rightFootTransform;
    std::string command = "";
    int i=0, imax=20;
    double heightDecrease = 0.1;
    double legVelKp = 0.01;

    /*Process:
        1. Get leg joint angles
        2. Get feet locations
        3. Adjust z value
        4. Get new leg joint angles
        5. Set joint velocities equal to error between desired angles and actual angles
        6. Use ankle compliance with velocity control */
   
    /* Main while loop */
    while(true)
    {
        hubo.update();

        i++; if(i>imax) i=0;


        /* Get joints angles for the legs */
        hubo.getLegAngles(LEFT, currentLeftLegAngles);
        hubo.getLegAngles(RIGHT, currentRightLegAngles);

        /* Get feet locations */
        hubo.huboLegFK(leftFootTransform, currentLeftLegAngles, LEFT);
        hubo.huboLegFK(rightFootTransform, currentRightLegAngles, RIGHT);

        /* Adjust height (z-value) */
        leftFootTransform(2,3) += heightDecrease;
        rightFootTransform(2,3) += heightDecrease;

        /* Get new leg joint angles */
        hubo.huboLegIK(refLeftLegAngles, leftFootTransform, currentLeftLegAngles, LEFT);
        hubo.huboLegIK(refRightLegAngles, rightFootTransform, currentRightLegAngles, RIGHT);

        /* Calculate error term */
        leftLegError = refLeftLegAngles - currentLeftLegAngles;
        rightLegError = refRightLegAngles - currentRightLegAngles;

        /* Set leg joint velocitys */
        hubo.setLegVels(LEFT, legVelKp*leftLegError);
        hubo.setLegVels(RIGHT, legVelKp*leftLegError);

        /* Send commands to Hubo */
        hubo.sendControls();

        // Display IMU readings
        if( i==imax )
        {
            std::cout << "\033[2J"
                    << "Left Leg Angles: " << currentLeftLegAngles.transpose()
                    << "\nRight Leg Angles: " << currentRightLegAngles.transpose()
                    << "\n\nLeft Leg Ref Vels: " << refLeftLegAngles.transpose()
                    << "\nRight Leg Ref Vels: " << refRightLegAngles.transpose()
                    << "\n\nLeft Leg Error: " << leftLegError.transpose()
                    << "\n\nRight Leg Error: " << rightLegError.transpose()
                    << "\n\nHeight Decrease: " << heightDecrease
                    << std::endl;
        }
    }
}
