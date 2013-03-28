#include <Hubo_Control.h>
#include <iostream>
#include <fstream>
#include <getopt.h>
#include "Fastrak.h"

#define FOOT_WIDTH = .130 // Width of Hubo's foot in meters

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
        "  -V, --verbose        Show output.\n"
        "  -H, --help           See this message\n";
}

/**
 * Main function that loops reading the sensors and commanding
 * Hubo's arm joints based on the poses of the hands
*/
int main(int argc, char **argv)
{

    // variables for the command line arguments
    bool print = false; // whether to print output or not
    bool left = false; // whether to set left arm angles
    bool right = false; // whether to set right arm angles

    // check if no arguments given, if not report usage
    if (argc != 2)
    {
        usage(std::cerr);
        return 1;
    }

    // command line lone options
    const struct option long_options[] = 
    {
        { "left",       no_argument, 0, 'l' },
        { "right",      no_argument, 0, 'r' },
        { "both",       no_argument, 0, 'b' },
        { "verbose",    no_argument, 0, 'V' },
        { "help",       no_argument, 0, 'H' },
        { 0,            0,           0,  0  },
    };

    // command line short options
    const char* short_options = "lrbVH";

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
            case 'V': print = true; break;
            case 'H': usage(std::cout); exit(0); break;
            default:  usage(std::cerr); exit(1); break;
        }
    }

    // LOCAL VARIABLES
    Vector6d initialLeftLegAngles, initialRightLegAngles;
    Vector6d refLeftLegAngles, refRightLegAngles;
    Vector6d leftLegError, rightLegError;
    Vector6d rActualAngles, rLegAnglesCurrent, rLegAnglesNext, checkr, legNomAcc;
    Vector6d lActualAngles, lLegAnglesNext, lLegAnglesCurrent, checkl;
    Vector3d lLegTrans, lFootInitialPos, lFastrakOrigin, lLegFastrak;
    Vector3d rLegTrans, rFootInitialPos, rFastrakOrigin, rLegFastrak; 
    Eigen::Matrix3d lRotOrigin, rRotOrigin, lRot, rRot;
    Eigen::Isometry3d lFootInitialPose, rFootInitialPose, lTransf, rTransf, lFootCurrent, rFootCurrent, B, Br;
    Eigen::Isometry3d leftFootTransform, rightFootTransform;
    double initialFootHeight = 0.1;
    double dt, ptime;
    int i=0, imax=50;
    int leftSensor=3; rightSensor=4;

    // OBJECTS
    // Create Hubo_Tech object
    Hubo_Control hubo;

    // Create Fastrak object
    Fastrak fastrak;
    // Initialize and start reading Fastrak
    fastrak.initFastrak();
    // Set Fastrak readings scale to 1:1
    fastrak.setFastrakScale( 1.0 );

    // Define arm nominal acceleration 
    legNomAcc << 0.3, 0.3, 0.3, 0.3, 0.3, 0.3;

    // Set arm nominal accelerations 
    hubo.setLeftLegNomAcc(legNomAcc);
    hubo.setRightLegNomAcc(legNomAcc);

    // Send commands to the control daemon 
    hubo.sendControls();

    // Get initial Fastrak sensors location and orientation
    fastrak.getPose(lFastrakOrigin, lRotOrigin, leftSensor, true);
    fastrak.getPose(rFastrakOrigin, rRotOrigin, rightSensor, false);

    // Get initial joint angles of the legs
    hubo.getLeftLegAngles(lLegAnglesNext);
    hubo.getRightLegAngles(rLegAnglesNext);

    // Get current pose of the foots
    hubo.huboLegFK(lFootInitialPose, lLegAnglesNext, LEFT);
    hubo.huboLegFK(rFootInitialPose, rLegAnglesNext, RIGHT);

    // Set relative zero for the foot locations
    lFootInitialPos = lFootInitialPose.translation();
    rFootInitialPos = rFootInitialPose.translation();

    // while the daemon is running
    while(!daemon_sig_quit)
    {
        // get latest state info for Hubo
        hubo.update();

        // check if new data was obtained from the ach channels
        dt = hubo.getTime() - ptime;
        ptime = hubo.getTime();

        // if new data is available...
        if(dt>0) {

            // get current joint angles
            hubo.getLeftLegAngles(lLegAnglesCurrent);
            hubo.getRightLegAngles(rLegAnglesCurrent);

            // get current foot locations and orientations
            hubo.huboLegFK(lFootCurrent, lLegAnglesCurrent, LEFT);    
            hubo.huboLegFK(rFootCurrent, rLegAnglesCurrent, RIGHT);    

            // get Fastrak data for left and right sensors
            fastrak.getPose( lLegFastrak, lRot, 1, true );
            fastrak.getPose( rLegFastrak, rRot, 2, false );

            // compute Fastrak relative translations
            lLegTrans = lLegFastrak - lFastrakOrigin;
            rLegTrans = rLegFastrak - rFastrakOrigin;

            // create 4d identity matrix
            lTransf = Eigen::Matrix4d::Identity();
            rTransf = Eigen::Matrix4d::Identity();

            // Create transformation matrix
            // pretranslate by the relative fastrak translation
            lTransf.translate(lLegTrans + lFootInitialPos);
            rTransf.translate(rLegTrans + rFootInitialPos);

            // make sure feet don't cross sagittal plane
            if(lTransf(1,3) - FOOT_WIDTH/2 < 0)
                lTransf(1,3) = FOOT_WIDTH/2;
            if(rTransf(1,3) + FOOT_WIDTH/2 > 0)
                rTransf(1,3) = -FOOT_WIDTH/2;

            // add rotation matrix to top-left corner
            lTransf.rotate(lRot);
            rTransf.rotate(rRot);

            // get joint angles corresponding to transformations
            hubo.huboLegIK( lLegAnglesNext, lTransf, lLegAnglesCurrent, LEFT );
            hubo.huboLegIK( rLegAnglesNext, rTransf, rLegAnglesCurrent, RIGHT );

            // set and get joint angles
            if( left==true )
            {
                hubo.setLeftLegAngles( lLegAnglesNext, false );
                hubo.getLeftLegAngles( lActualAngles );
            }

            if( right==true )
            {
                hubo.setRightLegAngles( rLegAnglesNext, false );
                hubo.getRightLegAngles( rActualAngles );
            }

            // send control references
            hubo.sendControls();

            // print data every i cycles
            if( i>=imax && print==true)
            {
                i = 0;
                std::cout << "Fastraktl: " << lLegTrans.transpose()
                          << "\nFastraktr: " << rLegTrans.transpose()
                          << "\nLEFTqd: " << lLegAnglesNext.transpose()
                          << "\nRIGTqd: " << rLegAnglesNext.transpose()
                          << std::endl;
            }
            i++;
        }
    }
}
