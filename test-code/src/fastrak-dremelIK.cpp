/* Description: This is a program is for drilling with the dremel strapped
                to Hubo's right hand. The drill bit position mimics the
                Fastrak sensor's y-position using a dedicated DrillIK function
                It uses hubo-motion-rt with the commands 'sudo service tech start' 
                instead of 'hubo-ach start'
*/

#include <Hubo_Tech.h>
#include <iostream>
#include <fstream>
#include "Fastrak.h"

using namespace std;

int main(int argc, char **argv)
{
    // Create Hubo_Tech object
    Hubo_Tech hubo;
    // Create Fastrak object
    Fastrak fastrak;
//    ofstream myfile;
//    myfile.open("ik-output.txt");
    // Initialize fastrak by opening fastrak ach channel
    fastrak.initFastrak();
    // Set fastrak scale to 1:1
    fastrak.setFastrakScale( 1.0 );

    hubo.setJointSpeedMax( RWP, 3.0 );
    hubo.setJointSpeedMax( LWP, 3.0 );

    // Local Variables
    Vector6d q, current;
    Eigen::Vector3d trans;
    Eigen::Quaterniond quat;
    double y=0, offset, dt, ptime=hubo.getTime();
    int i=0, imax=4; // loop count variable

    // Get joint angles for given y position of the drill
    hubo.HuboDrillIK( q, y );

    // Set RSR angle to -PI so his upper arm is parallel to the floor
    // in order to avoid having the drill bit hit anything
    q(2) -= M_PI/2.0;

    // Set right arm joint angles with new RSR angle and send commands
    hubo.setRightArmAngles( q, true );

    // Get current right arm joint angles 
    hubo.getRightArmAngles( current );

    // While the norm of the right arm angles is greater than 0.075
    // keep waiting for arm to get to desired position
    while( (current-q).norm() > 0.075 )
    {
        hubo.update(); // Get latest data from ach channels
        hubo.getRightArmAngles( current ); // Get current right arm joint angles
    }

    // Set RSR angle back to initial angle
    q(2) += M_PI/2.0;

    // Set right arm joint angles with new RSR angle and send commands
    hubo.setRightArmAngles( q, true );

    // Read fastrak sensor pose for sensor 1
    fastrak.getPose( trans, quat, 1, true );

    // Define initial y position to be the offset,
    // ie. take everything relative to that position
    //
    offset = trans(1);

    // If this daemon hasn't quit...
    while(!daemon_sig_quit)
    {
        // Get latest data from ach channels
        hubo.update();
        // Take latest time minus old time to see if a new time was received
        dt = hubo.getTime() - ptime;
        // Get current time in seconds
        ptime = hubo.getTime();

        // If new frame is received from hubo.update()
        if(dt>0)
        {
            // Increment i and reset it to 0 if it exceeds the max
            i++; if(i>imax) i=0;

            // Read Fastrak sensors
            fastrak.getPose( trans, quat, 1, true );

            // Set relative y-position
            y = trans(1) - offset;

            // Get joint angles for given y position of the drill
            hubo.HuboDrillIK( q, y );

            // Set joint angles
            hubo.setRightArmAngles( q );

            // Send commands to control daemon
            hubo.sendControls();

            // Print out useful info
            if( i==imax )
            {
                std::cout << "RH Mx" << hubo.getRightHandMx()
                          << "RH My" << hubo.getRightHandMy()
                          << "RH Angles:" << q.transpose()
                          << std::endl;
            }
        }
    }
}
