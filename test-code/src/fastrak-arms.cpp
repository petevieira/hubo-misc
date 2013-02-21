#include <Hubo_Tech.h>
#include <iostream>
#include <fstream>
#include "Fastrak.h"

int main(int argc, char **argv)
{
    Hubo_Tech hubo;
//    hubo_plus hubo("twoArmFastrak");

    // Create Fastrak object
    Fastrak fastrak;
    // Initialize Fastrak by opening fastrak ach channel
    fastrak.initFastrak();
    // Set Fastrak scale to 1:1
    fastrak.setFastrakScale( 1.0 );

    // Local Variables 
    Vector6d rActualAngles, rArmAnglesCurrent, rArmAnglesNext, checkr, armNomAcc;
    Vector6d lActualAngles, lArmAnglesNext, lArmAnglesCurrent, checkl;
    Vector3d lArmTrans, ltransEE, lArmCurrent, lArmFastrak;
    Vector3d rArmTrans, rtransEE, rArmCurrent, rArmFastrak; 
    Vector3d waistTrans, waistCurrent, waistFastrak;
    Eigen::Matrix3d lRotInitial, rRotInitial, lRot, rRot, waistRot, waistRotCurrent;
    Eigen::Isometry3d lcurrEE, rcurrEE, lTransf, rTransf, lHandCurrent, rHandCurrent, B, Br;
    double angle, waistPos, ptime, waistVel;
    double waistGain = 0.8;
    int i=0, imax=4;
    double dt;

    // Define arm nominal acceleration 
    armNomAcc << 0.3, 0.3, 0.3, 0.3, 0.3, 0.3;

    // Set arm nominal accelerations 
    hubo.setLeftArmNomAcc(armNomAcc);
    hubo.setRightArmNomAcc(armNomAcc);

    // Send commands to the control daemon 
    hubo.sendControls();

    // Get initial Fastrak sensors location and orientation
    fastrak.getPose( lArmCurrent, lRotInitial, 3, true );
    fastrak.getPose( rArmCurrent, rRotInitial, 4, false );

    // Define starting joint angles for the arms 
    lArmAnglesNext << 0, -.3, 0, -M_PI/2, 0, 0;
    rArmAnglesNext << 0, .3, 0, -M_PI/2, 0, 0;

    // Set the arm joint angles and send commands to the control daemon
    hubo.setLeftArmAngles( lArmAnglesNext, true );
    hubo.setRightArmAngles( rArmAnglesNext, true );

    // While the norm of the right arm angles is greater than 0.075
    // keep waiting for arm to get to desired position
    while ((lArmAnglesNext - checkl).norm() > 0.075 && (rArmAnglesNext - checkr).norm() > 0.075)
    {
        hubo.update(); // Get latest data from ach channels
        hubo.getLeftArmAngles(checkl); // Get current left arm joint angles
        hubo.getRightArmAngles(checkr); // Get current right arm joint angles
    }

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

        if(dt>0)
        {
            i++; if(i>imax) i=0;

            // get current joint angles
            hubo.getLeftArmAngles(lArmAnglesCurrent);
            hubo.getRightArmAngles(rArmAnglesCurrent);

            // get current hand locations and orientations
            hubo.huboArmFK(lHandCurrent, lArmAnglesCurrent, LEFT);    
            hubo.huboArmFK(rHandCurrent, rArmAnglesCurrent, RIGHT);    

            // get Fastrak data for left and right sensors
            fastrak.getPose( lArmFastrak, lRot, 3, true );
            fastrak.getPose( rArmFastrak, rRot, 4, false );

            // extract z-rotation angle from waistRot
    //        waistZRot = waistRot.eulerAngles(
    //        waistTrans = waistFastrak - waistCurrent;

            // compute Fastrak relative translations
            lArmTrans = lArmFastrak - lArmCurrent;
            rArmTrans = rArmFastrak - rArmCurrent;

            // compute Fastrak relative rotation
    //        lRot = lRotInitial.inv() * lRot;

            // create 4d identity matrix
            lTransf = Eigen::Matrix4d::Identity();
            rTransf = Eigen::Matrix4d::Identity();

            // Create transformation matrix
            // pretranslate by the relative fastrak translation
            lTransf.translate(lArmTrans + ltransEE);
            rTransf.translate(rArmTrans + rtransEE);
    //        lTransf.translate(ltransEE);
    //        lTransf.translate(lArmTrans);
    //        rTransf.translate(rArmTrans);

            // add rotation matrix to top-left corner
            lTransf.rotate(lRot);
            rTransf.rotate(rRot);

            // get final transformation for arms
    //        lTransf = lHandCurrent * lTransf;
    //        rTransf = rHandCurrent * rTransf;

            // get joint angles corresponding to transformations
            hubo.huboArmIK( lArmAnglesNext, lTransf, lArmAnglesCurrent, LEFT );
            hubo.huboArmIK( rArmAnglesNext, rTransf, rArmAnglesCurrent, RIGHT );
    //        angle = 90.0*M_PI/180.0;

//            waistVel = atan2(waistRot(1,0),waistRot(0,0));

//            hubo.setJointAngle(WST, waistVel*waistGain);

    //    	B(0,0) = 1; B(0,1) = 0; B(0,2) = 0; B(0,3) = ltransEE(0);
    //        B(1,0) = 0; B(1,1) = cos(angle); B(1,2) = -sin(angle); B(1,3) = ltransEE(1);
    //        B(2,0) = 0; B(2,1) = sin(angle); B(2,2) = cos(angle); B(2,3) = ltransEE(2);
    //        B(3,0) = 0; B(3,1) = 0; B(3,2) = 0; B(3,3) = 1;

    //    	hubo.huboArmIK( lArmAnglesNext, B, lArmAnglesCurrent, LEFT );
    //    	hubo.huboArmIK( lArmAnglesNext, lTransf, lArmAnglesCurrent, LEFT );
    //    	hubo.huboArmIK( rArmAnglesNext, rTransf, rArmAnglesCurrent, RIGHT );
    //    	hubo.huboArmIK( rArmAnglesNext, B, rArmAnglesCurrent, RIGHT );

            // set joint angles
            hubo.setLeftArmAngles( lArmAnglesNext, false );
            hubo.setRightArmAngles( rArmAnglesNext, false );
            hubo.getLeftArmAngles( lActualAngles);
            hubo.getRightArmAngles( rActualAngles);
    //        hubo.huboArmFK(lHandCurrent, lArmAnglesNext, LEFT);    
    //        hubo.huboArmFK(rHandCurrent, rArmAnglesNext, RIGHT);    

            // send control references
            hubo.sendControls();
            if( i==imax )
            {
/*                std::cout << "\033[2J"
                          << "Fastraktl: \n" << lArmTrans
                          << "\nFastrakl: \n" << lRot
                          << "Fastraktr: \n" << rArmTrans
                          << "\nFastrakr: \n" << rRot
                          << "\nlTransf: \n" << lTransf.matrix()
                          << "\nHuboOrient: \n" << lHandCurrent.matrix()
                          << "\nlArmAnglesl: " << lActualAngles.transpose()
                          << "\nrArmAnglesl: " << rActualAngles.transpose()
                          << "\nwaistLoc: " << waistTrans.transpose()
                          << "\nRight hand torques: " << hubo.getRightHandMx() << ", " << hubo.getRightHandMy()
                          << "\nLeft hand torques: " << hubo.getLeftHandMx() << ", " << hubo.getLeftHandMy()
                          << std::endl;
                          i = 0;
*/            }
        }
    }
}
