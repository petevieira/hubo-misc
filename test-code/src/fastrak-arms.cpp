#include <Hubo_Control.h>
#include <iostream>
#include <fstream>
#include <getopt.h>
#include "Fastrak.h"

/**
 * Prints out how to run this program
*/
void usage(std::ostream& ostr) {
    ostr << 
        "usage: fastrak-arms [OPTIONS] \n"
        "\n"
        "OPTIONS:\n"
        "\n"
        "  -l, --left           Control left arm only.\n"
        "  -r, --right          Control right arm only.\n"
        "  -b, --both           Control both arms.\n"
        "  -n, --nosend         Don't send commands to Hubo.\n"
        "  -V, --verbose        Show output.\n"
        "  -H, --help           See this message\n";
}

/**
 * Main function that loops reading the sensors and commanding
 * Hubo's arm joints based on the poses of the hands
*/
int main(int argc, char **argv)
{
    // check if no arguments given, if not report usage
    if (argc < 2)
    {
        usage(std::cerr);
        return 1;
    }

    bool print = false; // whether to print output or not
    bool left = false; // whether to set left arm angles
    bool right = false; // whether to set right arm angles
    bool send = true; // whether to send commands or not

    // command line lone options
    const struct option long_options[] = 
    {
        { "left",       no_argument, 0, 'l' },
        { "right",      no_argument, 0, 'r' },
        { "both",       no_argument, 0, 'b' },
        { "nosend",     no_argument, 0, 'n' },
        { "verbose",    no_argument, 0, 'V' },
        { "help",       no_argument, 0, 'H' },
        { 0,            0,           0,  0  },
    };

    // command line short options
    const char* short_options = "lrbnVH";

    // command line option and option index number
    int opt, option_index;

    // loop through command line options and set values accordingly
    while ( (opt = getopt_long(argc, argv, short_options, long_options, &option_index)) != -1 )
    {
        switch (opt)
        {
            case 'l': left = true; break;
            case 'r': right = true; break;
            case 'b': left = true; right = true; break;
            case 'n': send = false; break;
            case 'V': print = true; break;
            case 'H': usage(std::cout); exit(0); break;
            default:  usage(std::cerr); exit(1); break;
        }
    }

    // Create Hubo_Control object
    Hubo_Control hubo;
//    Hubo_Control hubo("fastrak-arms");

    // Create Fastrak object
    Fastrak fastrak;
    // Initialize Fastrak by opening fastrak ach channel
    fastrak.initFastrak();
    // Set Fastrak scale to 1:1
    fastrak.setFastrakScale( 1.0 );

    // Local Variables 
    Vector6d rActualAngles, rArmAnglesCurrent, rArmAnglesNext, checkr, armNomAcc;
    Vector6d lActualAngles, lArmAnglesNext, lArmAnglesCurrent, checkl;
    Vector3d lArmTrans, ltransEE, lTransInitial, lArmFastrak;
    Vector3d rArmTrans, rtransEE, rTransInitial, rArmFastrak; 
    Eigen::Matrix3d lRotInitial, rRotInitial, lRot, rRot;
    Eigen::Isometry3d lcurrEE, rcurrEE, lTransf, rTransf, lHandCurrent, rHandCurrent;
    int i=0, imax=40;
    double dt, ptime;
    int leftSensor=2, rightSensor=5;

    // Define arm nominal acceleration 
    armNomAcc << 0.3, 0.3, 0.3, 0.3, 0.3, 0.3;

    // Set arm nominal accelerations 
    hubo.setLeftArmNomAcc(armNomAcc);
    hubo.setRightArmNomAcc(armNomAcc);

    // Send commands to the control daemon 
    hubo.sendControls();

    // Get initial Fastrak sensors location and orientation
    fastrak.getPose( lTransInitial, lRotInitial, leftSensor, true );
    fastrak.getPose( rTransInitial, rRotInitial, rightSensor, false );

    // Define starting joint angles for the arms 
//    lArmAnglesNext << 0, -.3, 0, -M_PI/2, 0, 0;
//    rArmAnglesNext << 0, .3, 0, -M_PI/2, 0, 0;

    // Set the arm joint angles and send commands to the control daemon
//    hubo.setLeftArmAngles( lArmAnglesNext, true );
//    hubo.setRightArmAngles( rArmAnglesNext, true );

    // While the norm of the right arm angles is greater than 0.075
    // keep waiting for arm to get to desired position
//    while ((lArmAnglesNext - checkl).norm() > 0.075 && (rArmAnglesNext - checkr).norm() > 0.075)
//    {
//        hubo.update(); // Get latest data from ach channels
//        hubo.getLeftArmAngles(checkl); // Get current left arm joint angles
//        hubo.getRightArmAngles(checkr); // Get current right arm joint angles
//    }

    // Get current pose of the hands
    hubo.huboArmFK(lcurrEE, lArmAnglesNext, LEFT);
    hubo.huboArmFK(rcurrEE, rArmAnglesNext, RIGHT);

    // Set relative zero for the hand locations
    ltransEE = lcurrEE.translation();
    rtransEE = rcurrEE.translation();

    while(!daemon_sig_quit)
    {
        // get latest state info for Hubo
        hubo.update();

        dt = hubo.getTime() - ptime;
        ptime = hubo.getTime();

        // if new data is available...
        if(dt>0)
        {
            // get current joint angles
            hubo.getLeftArmAngles(lArmAnglesCurrent);
            hubo.getRightArmAngles(rArmAnglesCurrent);

            // get current hand locations and orientations
            hubo.huboArmFK(lHandCurrent, lArmAnglesCurrent, LEFT);    
            hubo.huboArmFK(rHandCurrent, rArmAnglesCurrent, RIGHT);    

            // get Fastrak data for left and right sensors
            fastrak.getPose( lArmFastrak, lRot, leftSensor, true );
            fastrak.getPose( rArmFastrak, rRot, rightSensor, false );

            // compute Fastrak relative translations
            lArmTrans = lArmFastrak - lTransInitial;
            rArmTrans = rArmFastrak - rTransInitial;

            // create 4d identity matrix
            lTransf = Eigen::Matrix4d::Identity();
            rTransf = Eigen::Matrix4d::Identity();

            // Create transformation matrix
            // pretranslate by the relative fastrak translation
            lTransf.translate(lArmTrans + ltransEE);
            rTransf.translate(rArmTrans + rtransEE);

            // add rotation matrix to top-left corner
            lTransf.rotate(lRot);
            rTransf.rotate(rRot);

            // get joint angles corresponding to transformations
            hubo.huboArmIK( lArmAnglesNext, lTransf, lArmAnglesCurrent, LEFT );
            hubo.huboArmIK( rArmAnglesNext, rTransf, rArmAnglesCurrent, RIGHT );

            // set and get joint angles
            if( left==true )
            {
                hubo.setLeftArmAngles( lArmAnglesNext, false );
                hubo.getLeftArmAngles( lActualAngles );
            }

            if( right==true )
            {
                hubo.setRightArmAngles( rArmAnglesNext, false );
                hubo.getRightArmAngles( rActualAngles );
            }

            // send control references
            if( send == true )
            {
                hubo.sendControls();
            }

            // print output every imax cycles
            if( i>=imax && print==true )
            {
                std::cout //<< "\033[2J"
                          << "Fastrak Position Lt(m): \n" << lArmTrans
                          << "Fastrak Position Rt(m): \n" << rArmTrans
                          << "\nLeft  Arm Angles(rad): " << lActualAngles.transpose()
                          << "\nRight Arm Angles(rad): " << rActualAngles.transpose()
                          << "\nRight hand torques(N-m): " << hubo.getRightHandMx() << ", " << hubo.getRightHandMy()
                          << "\nLeft  hand torques(N-m): " << hubo.getLeftHandMx() << ", " << hubo.getLeftHandMy()
                          << std::endl;
            }
            if(i>=imax) i=0; i++;
        }
    }
}
