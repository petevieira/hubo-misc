#include "hubo_plus.h"
#include <iostream>
#include <fstream>
using namespace std;

int main(int argc, char **argv)
{
    hubo_plus hubo("ik-test");
//    ofstream myfile;
//    myfile.open("ik-output.txt");
    hubo.initFastrak();
    hubo.setFastrakScale( 1.0 );

    Vector6d q, current;
    double y=0, offset, dt, ptime=hubo.getTime();
    int i=0, imax=4;

    hubo.HuboDrillIK( q, y );
    q(2) -= M_PI/2.0;
    hubo.setRightArmAngles( q, true );
    
    hubo.getRightArmAngles( current );
    while( (current-q).norm() > 0.075 )
    {
        hubo.update();
        hubo.getRightArmAngles( current );
    }
    q(2) += M_PI/2.0;
    hubo.setRightArmAngles( q, true );


    Eigen::Vector3d trans; Eigen::Quaterniond quat;
    hubo.getFastrak( trans, quat, 1, true );
    offset = trans(1);

    

    
    std::cout << "Time,y,Mx,My,q1,q2,q3,q4,q5,q6" << std::endl;

    while(!daemon_sig_quit)
    {
        hubo.update();
        dt = hubo.getTime() - ptime;
        ptime = hubo.getTime();

        if(dt>0)
        {
            i++; if(i>imax) i=0;
            hubo.getFastrak( trans, quat, 1, true );
            y = trans(1) - offset;

            hubo.HuboDrillIK( q, y );
            hubo.setRightArmAngles( q, true );
            
            if( i==imax )
                std::cout <<  ptime << "," << y << "," << hubo.getRightHandMx() << "," << hubo.getRightHandMy() << "," << q(0) << "," << q(1) << "," << q(2) << "," << q(3) << "," << q(4) << "," << q(5) << std::endl;
        }
    }


/*
    // Testing consistency of FK and IK
    Vector6d qprev, q0l, q0r, q0;
    q0l << 0, 0, 0, -M_PI/2.0, 0.3, 0;
    q0r << 0, 0, 0, 0, 0, 0;
    
    std::cout << "Left commands: " << q0l.transpose() << std::endl;
    std::cout << "Right commands: " << q0r.transpose() << std::endl;
    

    Eigen::Matrix4d Bleft, Bright;
    

    //hubo.huboArmFK( Bleft, q0l, LEFT );
    //hubo.huboArmFK( Bright, q0r, RIGHT );


    Bleft <<    0.679443,    -0.66597,    0.307964, 9.83477e-06,
   0.316599,    0.644741,    0.695755,    0.335909,
  -0.661909,   -0.375225,     0.64891,    -0.39233,
          0,           0,           0,           1;

    std::cout << Bleft.inverse() << std::endl;

    hubo.huboArmIK( q0l, (Eigen::Isometry3d)Bleft, q0l, LEFT );
    //hubo.huboArmIK( q0r, Bright, q0r, RIGHT );

    Eigen::Vector3d z(0, 0, 1);

    
    std::cout << "Left result: " << q0l.transpose() << std::endl;
    std::cout << "Right result: " << q0r.transpose() << std::endl;

//    std::cout << q0.transpose() << std::endl;

    std::cout << "Left arm:" << std::endl;
    std::cout << Bleft.matrix() << std::endl;

    std::cout << "_____________\n" << "Right arm:" << std::endl;
    std::cout << Bright.matrix() << std::endl;


*/


/*
    double y=0;
    double y0=-0.2145;

    Vector6d q, ql, qr, q0; q.setZero(); q0.setZero();
    ql << 0, 0, 0, -M_PI/2, 0.3, 0;
    qr << 0, 0, 0, -M_PI/2, -0.3, 0;

    hubo.setLeftArmAngles( ql );
    hubo.setRightArmAngles( qr );
    hubo.sendControls();

    Vector6d langlecheck, ranglecheck;

    hubo.update();
    hubo.getLeftArmAngleStates( langlecheck );
    hubo.getRightArmAngleStates( ranglecheck );
    while(false)
//    while( (langlecheck-ql).norm() > 0.075 && (ranglecheck-qr).norm() > 0.075 )
    {
        hubo.update();
        hubo.getLeftArmAngleStates( langlecheck );
        hubo.getRightArmAngleStates( ranglecheck );
    }

    Eigen::Vector3d lsensorOS, rsensorOS, lhandOS, rhandOS, pos;
    Eigen::Quaterniond quat;
    Eigen::Isometry3d ltf, rtf, B;

    hubo.getFastrak( lsensorOS, quat, 1, true );
    hubo.getFastrak( rsensorOS, quat, 2, false );
    hubo.sendControls();

    hubo.huboArmFK( B, ql, LEFT );
    lhandOS = B.translation();
    hubo.huboArmFK( B, qr, RIGHT );
    rhandOS = B.translation();

    
    hubo.HuboDrillIK( q, y );
    hubo.HuboDrillFK( B, q );
    std::cout << B.matrix() << std::endl;
    std::cout << q.transpose() << std::endl;
    std::cout << B(1,3) - y0 << std::endl;
    

    std::cout << "Left Hand offset: " << lhandOS.transpose() << std::endl;
    std::cout << "Left Sensor offset: " << lsensorOS.transpose() << std::endl;
    

//    Eigen::Vector3d position;
//    Eigen::Matrix3d rotation;
    while(false)
    {
        hubo.update();


        ltf = Eigen::Matrix4d::Identity();
        rtf = Eigen::Matrix4d::Identity();

        hubo.getFastrak( ltf, 1, true );
        hubo.getFastrak( rtf, 2, false );
        ltf.pretranslate( lhandOS-lsensorOS );
        rtf.pretranslate( rhandOS-lsensorOS );

        std::cout << ltf.translation().transpose() << "\t\t\t";

//        std::cout << tf.translation().transpose()-handOS.transpose() << "\t\t\t";
//        hubo.getLeftArmAngleStates( q );
 
        hubo.huboArmIK( ql, ltf, q0, LEFT );
        hubo.huboArmIK( qr, rtf, q0, RIGHT );

        hubo.setLeftArmAngles( ql );
        hubo.setRightArmAngles( qr );

        hubo.sendControls();
        
        std::cout << q.transpose() << std::endl;

    }*/
}
