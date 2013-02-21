#include <Hubo_Tech.h>
#include <iostream>
#include <fstream>
#include "Fastrak.h"

int main(int argc, char **argv)
{
    std::cout << "start\n";
    Hubo_Tech hubo;
    Fastrak fastrak;
    fastrak.initFastrak();
    fastrak.setFastrakScale( 1.0 );

    Vector6d rActualAngles, rArmAnglesCurrent, rArmAnglesNext, checkr, legNomAcc;
    Vector6d lActualAngles, lArmAnglesNext, lArmAnglesCurrent, checkl;
    Vector3d lArmTrans, ltransEE, lArmCurrent, lArmFastrak;
    Vector3d rArmTrans, rtransEE, rArmCurrent, rArmFastrak; 
    Vector3d waistTrans, waistCurrent, waistFastrak;
    Eigen::Matrix3d lRotInitial, rRotInitial, lRot, rRot, waistRot, waistRotCurrent;
    Eigen::Isometry3d lcurrEE, rcurrEE, lTransf, rTransf, lHandCurrent, rHandCurrent, B, Br;

    double angle, waistPos, ptime, waistVel;

    Vector6d currentLeftLegAngles, currentRightLegAngles;
    Vector6d refLeftLegAngles, refRightLegAngles;
    Vector6d leftLegError, rightLegError;
    Eigen::Isometry3d leftFootTransform, rightFootTransform;

    double waistGain = 0.8;
    int i=0, imax=50;
    double heightDecrease = 0.1;
    double dt;

    // Define arm nominal acceleration 
    legNomAcc << 0.3, 0.3, 0.3, 0.3, 0.3, 0.3;

    // Set arm nominal accelerations 
    hubo.setLeftLegNomAcc(legNomAcc);
    hubo.setRightLegNomAcc(legNomAcc);

    // Send commands to the control daemon 
    hubo.sendControls();

    hubo.getLegAngles(LEFT, currentLeftLegAngles);
    hubo.getLegAngles(RIGHT, currentRightLegAngles);

    // Get feet locations
    hubo.huboLegFK(leftFootTransform, currentLeftLegAngles, LEFT);
    hubo.huboLegFK(rightFootTransform, currentRightLegAngles, RIGHT);

    // Adjust height (z-value)
    leftFootTransform(2,3) += heightDecrease;
    rightFootTransform(2,3) += heightDecrease;

    // Get new leg joint angles
    hubo.huboLegIK(lArmAnglesNext, leftFootTransform, currentLeftLegAngles, LEFT);
    hubo.huboLegIK(rArmAnglesNext, rightFootTransform, currentRightLegAngles, RIGHT);
    
    hubo.setLeftLegAngles( lArmAnglesNext, false );
    hubo.setRightLegAngles( rArmAnglesNext, false );

    hubo.sendControls();

   while ( (rArmAnglesNext - checkr).norm() > 0.075 && (lArmAnglesNext - checkl).norm() > 0.075 )
    {
        hubo.update(); // Get latest data from ach channels
        hubo.getLegAngles(LEFT, checkl);
        hubo.getLegAngles(RIGHT, checkr);
        std::cout << "checkr: " << checkr.transpose() << std::endl;
        std::cout << "rArm  : " << rArmAnglesNext.transpose() << std::endl;
    }

    printf("Exited position intializaiton loop\n");
    // Get initial Fastrak sensors location and orientation
    fastrak.getPose( lArmCurrent, lRotInitial, 1, true );
    fastrak.getPose( rArmCurrent, rRotInitial, 2, false );

//    fastrak.getPose( waistCurrent, waistRotCurrent, 3, false );

    hubo.getLeftLegAngles( lArmAnglesNext);
    hubo.getRightLegAngles( rArmAnglesNext);

    // Get current pose of the hands
    hubo.huboLegFK(lcurrEE, lArmAnglesNext, LEFT);
    hubo.huboLegFK(rcurrEE, rArmAnglesNext, RIGHT);

    // Set relative zero for the hand locations
    ltransEE = lcurrEE.translation();
    rtransEE = rcurrEE.translation();

    while(!daemon_sig_quit)
    {
        // get latest state info for Hubo

        hubo.update();

        dt = hubo.getTime() - ptime;
        ptime = hubo.getTime();

        if(dt>0) {
       
            i++; if(i>imax) i=0;

            // get current joint angles
            hubo.getLeftLegAngles(lArmAnglesCurrent);
            hubo.getRightLegAngles(rArmAnglesCurrent);

            // get current hand locations and orientations
            hubo.huboLegFK(lHandCurrent, lArmAnglesCurrent, LEFT);    
            hubo.huboLegFK(rHandCurrent, rArmAnglesCurrent, RIGHT);    

            // get Fastrak data for left and right sensors
            fastrak.getPose( lArmFastrak, lRot, 1, true );
            fastrak.getPose( rArmFastrak, rRot, 2, false );
//            fastrak.getPose( waistFastrak, waistRot, 3, false );

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
            hubo.huboLegIK( lArmAnglesNext, lTransf, lArmAnglesCurrent, LEFT );
            hubo.huboLegIK( rArmAnglesNext, rTransf, rArmAnglesCurrent, RIGHT );
    //        angle = 90.0*M_PI/180.0;

//            waistVel = atan2(waistRot(1,0),waistRot(0,0));

//            hubo.setJointAngle(WST, waistVel*waistGain);

            // set joint angles
            hubo.setLeftLegAngles( lArmAnglesNext );
            hubo.setRightLegAngles( rArmAnglesNext );
            hubo.getLeftLegAngles( lActualAngles );
            hubo.getRightLegAngles( rActualAngles );
    //        hubo.huboArmFK(lHandCurrent, lArmAnglesNext, LEFT);    
    //        hubo.huboArmFK(rHandCurrent, rArmAnglesNext, RIGHT);    

            // send control references
            hubo.sendControls();
            if( i==imax )
            {
                std::cout // "\033[2J"
                          << "Fastraktl: " << lArmTrans.transpose()
            //              << "\nFastrakl: \n" << lRot
                          << "\nFastraktr: " << rArmTrans.transpose()
            //              << "\nFastrakr: \n" << rRot
            //              << "\nlTransf: \n" << lTransf.matrix()
            //              << "\nHuboOrient: \n" << lHandCurrent.matrix()
            //              << "\n\nlArmAngles: " << lArmAnglesNext.transpose()
            //              << "\nrFootAngles: " << rActualAngles.transpose()
                          << "\nLEFTqd: " << rArmAnglesNext.transpose()
                          << "\nRIGTqd: " << lArmAnglesNext.transpose()
                          << std::endl;
            }
        }
    }
}
