#include "hubo_plus.h"
#include <iostream>
#include <fstream>

int main(int argc, char **argv)
{
    hubo_plus hubo;
//    hubo_plus hubo("twoArmFastrak");
    hubo.initFastrak();
    hubo.setFastrakScale( 1.0 );

    double waistVel;
    Vector6d lArmAnglesNext, rArmAnglesNext, lArmAnglesCurrent, rArmAnglesCurrent;
    Vector3d lArmTrans, rArmTrans, lArmCurrent, rArmCurrent, lArmFastrak, rArmFastrak, waistFastrak;
    Eigen::Matrix3d lRotInitial, rRotInitial, lRot, rRot, waistRot;
    Eigen::Isometry3d lTransf, rTransf, lHandCurrent, rHandCurrent;

    // get initial Fastrak location and orientation
    hubo.getFastrak( lArmCurrent, lRotInitial, 1, true );
//    hubo.getFastrak( rArmCurrent, rRotCurrent, 2, false );
//    hubo.getFastrak( waistCurrent, waistRotCurrent, 3, false );

    while(!daemon_sig_quit)
    {
        // get latest state info for Hubo
        hubo.update();

        // get current joint angles
        hubo.getLeftArmAngles(lArmAnglesCurrent);
//       hubo.getRightArmAngles(rArmAnglesCurrent);

        // get current hand locations and orientations
        hubo.huboArmFK(lHandCurrent, lArmAnglesCurrent, LEFT);    
//        hubo.huboArmFK(rHandCurrent, rArmAnglesCurrent, RIGHT);    

        // get Fastrak data for left and right sensors
        hubo.getFastrak( lArmFastrak, lRot, 1, true );
//        hubo.getFastrak( rArmFastrak, rRot, 2, false );
//        hubo.getFastrak( waistFastrak, waistRot, 3, false );

        // extract z-rotation angle from waistRot
//        waistZRot = waistRot.eulerAngles(

        // compute Fastrak relative translations
        lArmTrans = lArmFastrak - lArmCurrent;
//        rArmTrans = rArmFastrak - rArmCurrent;

        // compute Fastrak relative rotation
//        lRot = lRotInitial.inv() * lRot;

        // create 4d identity matrix
        lTransf = Eigen::Matrix4d::Identity();
//        rTransf = Eigen::Matrix4d::Identity();

        // Create transformation matrix
        // pretranslate by the relative fastrak translation
        lTransf.translate(lArmTrans);
//        rTransf.translate(rArmTrans);

        // add rotation matrix to top-left corner
        lTransf.rotate(lRot);
//        rTransf.rotate(rRot);

        // get final transformation for arms
        lTransf = lHandCurrent * lTransf;
//        rTransf = rHandCurrent * rTransf;

        // get joint angles corresponding to transformations
        hubo.huboArmIK( lArmAnglesNext, lTransf, lArmAnglesCurrent, LEFT );
//        hubo.huboArmIK( rArmAnglesNext, rTransf, rArmAnglesCurrent, RIGHT );

        // set joint angles
        hubo.setLeftArmAngles( lArmAnglesNext, false );
//        hubo.setRightArmAngles( rArmAnglesNext, false );

        // send control references
        hubo.sendControls();

        // print data
//        std::cout << std::endl;
    }
}
