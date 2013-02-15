/* Description: This is a squatting program to test the leg foward and inverse
 * kinematics. It uses 'sudo service tech start' instead of 'hubo-ach start'
*/

#include <Hubo_Tech.h>
#include <iostream>

 
int main(int argc, char **argv)
{
    // Instantiate Hubo_Tech object from constructor
    Hubo_Tech hubo;

    // Set Left Leg Joint Nominal Accelerations
    hubo.setJointNominalAcceleration( LKN, 0.6 );
    hubo.setJointNominalAcceleration( LHP, 0.6 );
    hubo.setJointNominalAcceleration( LAR, 0.6 );
    hubo.setJointNominalAcceleration( LAP, 0.6 );

    // Set Right Leg Joint Nominal Accelerations
    hubo.setJointNominalAcceleration( RKN, 0.6 );
    hubo.setJointNominalAcceleration( RHP, 0.6 );
    hubo.setJointNominalAcceleration( RHR, 0.6 );
    hubo.setJointNominalAcceleration( RHY, 0.6 );
    hubo.setJointNominalAcceleration( RAP, 0.6 );
    hubo.setJointNominalAcceleration( RAR, 0.6 );

    // Local Variables
    Vector6d currentLeftLegAngles, currentRightLegAngles;
    Vector6d refLeftLegAngles, refRightLegAngles;
    Vector6d leftLegError, rightLegError;
    Eigen::Isometry3d leftFootTransform, rightFootTransform;
    std::string command = "";
    int i=0, imax=40;
    double heightDecrease = 0.1;
    double legVelKp = 0.1;
    double ptime, dt;

    // Get latest data from the ach channels
    hubo.update();

    // Get current time
    ptime = hubo.getTime();

    /*Process:
        1. Get leg joint angles
        2. Get feet locations
        3. Adjust z value
        4. Get new leg joint angles
        5. Set joint velocities equal to error between desired angles and actual angles
        6. Use ankle compliance with velocity control */

    // Get joints angles for the legs
    hubo.getLegAngles(RIGHT, currentRightLegAngles);

    // Get feet locations
    hubo.huboLegFK(rightFootTransform, currentRightLegAngles, RIGHT);

    // Adjust height (z-value)
    rightFootTransform(2,3) += heightDecrease;

    // Get new leg joint angles
    hubo.huboLegIK(refRightLegAngles, rightFootTransform, currentRightLegAngles, RIGHT);

    // Main while loop
    while(true)
    {
        // Get latest data from the ach channels
        hubo.update();

        dt = hubo.getTime() - ptime;
        ptime = hubo.getTime();
        if( dt > 0 )
        {
            i++; if(i>imax) i=0;

            // Calculate error term
            rightLegError = refRightLegAngles - hubo.getLegAngles(RIGHT, currentRightLegAngles);

            // Set leg joint velocitys
            hubo.setLegVels(RIGHT, legVelKp*leftLegError);

            // Send commands to Hubo
            hubo.sendControls();

            // Display IMU readings
            if( i==imax )
            {
                std::cout
                        << "Right Leg Angles: " << currentRightLegAngles.transpose()
                        << "\nRight Leg Ref Vels: " << refRightLegAngles.transpose()
                        << "\n\nRight Leg Error: " << rightLegError.transpose()
                        << "\n\nHeight Decrease: " << heightDecrease
                        << std::endl;
            }
        }
    }
}
