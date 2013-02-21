#include <Hubo_Tech.h>
#include <iostream>
#include <fstream>
#include "Fastrak.h"

int main(int argc, char **argv)
{
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

    // OBJECTS
    // Create Hubo_Tech object
    Hubo_Tech hubo;

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

    hubo.getLegAngles(LEFT, initialLeftLegAngles);
    hubo.getLegAngles(RIGHT, initialRightLegAngles);

    // Get feet locations
    hubo.huboLegFK(leftFootTransform, initialLeftLegAngles, LEFT);
    hubo.huboLegFK(rightFootTransform, initialRightLegAngles, RIGHT);

    // Adjust height (z-value)
    leftFootTransform(2,3) += initialFootHeight;
    rightFootTransform(2,3) += initialFootHeight;

    // Get new leg joint angles
    hubo.huboLegIK(lLegAnglesNext, leftFootTransform, initialLeftLegAngles, LEFT);
    hubo.huboLegIK(rLegAnglesNext, rightFootTransform, initialRightLegAngles, RIGHT);

    // set leg angles to initial positions
    hubo.setLeftLegAngles( lLegAnglesNext, false );
    hubo.setRightLegAngles( rLegAnglesNext, false );

    // Send commands to the control daemon
    hubo.sendControls();

    // wait till the legs get to the initial positions
    while((rLegAnglesNext - checkr).norm() > 0.075 && (lLegAnglesNext - checkl).norm() > 0.075)
    {
        hubo.update(); // Get latest data from ach channels
        hubo.getLegAngles(LEFT, checkl);
        hubo.getLegAngles(RIGHT, checkr);
    }

    // Get initial Fastrak sensors location and orientation
    fastrak.getPose(lFastrakOrigin, lRotOrigin, 1, true);
    fastrak.getPose(rFastrakOrigin, rRotOrigin, 2, false);

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

            // increment counter for printing
            i++;

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

            // add rotation matrix to top-left corner
            lTransf.rotate(lRot);
            rTransf.rotate(rRot);

            // get joint angles corresponding to transformations
            hubo.huboLegIK( lLegAnglesNext, lTransf, lLegAnglesCurrent, LEFT );
            hubo.huboLegIK( rLegAnglesNext, rTransf, rLegAnglesCurrent, RIGHT );

            // set joint angles
            hubo.setLeftLegAngles( lLegAnglesNext );
            hubo.setRightLegAngles( rLegAnglesNext );

            // get current joint angles
            hubo.getLeftLegAngles( lActualAngles );
            hubo.getRightLegAngles( rActualAngles );

            // send control references
            hubo.sendControls();

            // print data every i cycles
            if( i>=imax )
            {
                i = 0;
/*                std::cout << "Fastraktl: " << lLegTrans.transpose()
                          << "\nFastraktr: " << rLegTrans.transpose()
                          << "\nLEFTqd: " << lLegAnglesNext.transpose()
                          << "\nRIGTqd: " << rLegAnglesNext.transpose()
                          << std::endl;
*/            }
        }
    }
}
