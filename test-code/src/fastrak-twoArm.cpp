#include "hubo_plus.h"
#include <iostream>
#include <fstream>

int main(int argc, char **argv)
{
    hubo_plus hubo;
//    hubo_plus hubo("twoArmFastrak");
    hubo.initFastrak();
    hubo.setFastrakScale( 1.0 );

    int i=0, imax=4;
    double angle, waistPos, ptime, waistVel;
    Vector6d rActualAngles, lActualAngles, lAcc, lArmAnglesNext, rArmAnglesNext, checkl, checkr, lArmAnglesCurrent, rArmAnglesCurrent;
    Vector3d lArmTrans, rArmTrans, ltransEE, rtransEE, lArmCurrent, rArmCurrent, lArmFastrak, rArmFastrak, waistFastrak, waistTrans, waistCurrent;
    Eigen::Matrix3d lRotInitial, rRotInitial, lRot, rRot, waistRot, waistRotCurrent;
    Eigen::Isometry3d lcurrEE, rcurrEE, lTransf, rTransf, lHandCurrent, rHandCurrent, B, Br;

    lAcc << 0.3, 0.3, 0.3, 0.3, 0.3, 0.3;
    double waistGain = 0.8;
    hubo.setLeftArmNomAcc(lAcc);
    hubo.setRightArmNomAcc(lAcc);
    hubo.sendControls();
    // get initial Fastrak location and orientation
    fastrak.getPose( lArmCurrent, lRotInitial, 1, true );
    fastrak.getPose( rArmCurrent, rRotInitial, 2, false );
    fastrak.getPose( waistCurrent, waistRotCurrent, 3, false );

    lArmAnglesNext << 0, -.3, 0, -M_PI/2, 0, 0;
    rArmAnglesNext << 0, .3, 0, -M_PI/2, 0, 0;
    hubo.setLeftArmAngles( lArmAnglesNext, true );
    hubo.setRightArmAngles( rArmAnglesNext, true );
    while ((lArmAnglesNext - checkl).norm() > 0.075 && (rArmAnglesNext - checkr).norm() > 0.075)
    {
        hubo.update();
        hubo.getLeftArmAngles(checkl);
        hubo.getRightArmAngles(checkr);
    }
    hubo.huboArmFK(lcurrEE, lArmAnglesNext, LEFT);
    hubo.huboArmFK(rcurrEE, rArmAnglesNext, RIGHT);
    ltransEE = lcurrEE.translation();
    rtransEE = rcurrEE.translation();

    std::cout << "Time,WSTVel,WSTAngle,lq1,lq2,lq3,lq4,lq5,lq6,rq1,rq2,rq3,rq4,rq5,rq6" << std::endl;

    while(!daemon_sig_quit)
    {
        i++;
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
        fastrak.getPose( lArmFastrak, lRot, 1, true );
        fastrak.getPose( rArmFastrak, rRot, 2, false );
        fastrak.getPose( waistFastrak, waistRot, 3, false );

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

        waistVel = atan2(waistRot(1,0),waistRot(0,0));

        hubo.setJointAngle(WST, waistVel*waistGain);

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
    std::cout <<  ptime << "," << waistVel << "," << hubo.getJointAngle(WST) << "," << "," << lActualAngles(0) << "," << lActualAngles(1) << "," << lActualAngles(2) << "," << lActualAngles(3) << "," << lActualAngles(4) << "," << lActualAngles(5) << "," << rActualAngles(0) << "," << rActualAngles(1) << "," << rActualAngles(2) << "," << rActualAngles(3) << "," << rActualAngles(4) << "," << rActualAngles(5) << std::endl;
//            std::cout << waistRot << "\nWSTVelRef: " << waistVel << "\nwaistAngle: " << hubo.getJointAngle(WST) << "\nWSTVel: " << hubo.getJointVelocity(WST) << "\n" << std::endl;
            i=0;
            }
/*     if(i >  10)
        {
    	std::cout << "\033[2J"
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
        }
*/    }
}
