#include "hubo_plus.h"
#include <iostream>

 
int main(int argc, char **argv)
{
    hubo_plus hubo;
    int i=0, imax=20;
    Vector6d angles;
    Vector3d legCOM;

    hubo.setJointAngle(RKN, .5);
    hubo.sendControls();

    while(true)
    {
        hubo.update();
        usleep(100000);
        hubo.getLeftLegAngleStates(angles);
        legCOM = hubo.getCOMOfLeg(RIGHT);
        std::cout << "COM: " << legCOM.transpose() 
                  << "Angles: " << angles.transpose()
                  << std::endl;
    }
    
    return 0;
}

// Function, which goes in Hubo_Tech.cpp and Hubo_Tech.h
/*
Vector3d hubo_plus::getCOMOfLeg(int side) {

    Vector3d legCOM;
    std::vector<Eigen::Vector3d> pCOM(6);
    double sumMasses;
    Eigen::Isometry3d jointLoc;
    Vector6d legAngles, mass;
    Vector3d sumMp;
    sumMp << 0, 0, 0;

    pCOM[0] << -0.0347, 0.000, 0.072;
    pCOM[1] << 0.049747912596, 0.012531116599, 0.015643664572;
    pCOM[2] << -0.019504891350, -0.059577480789, 0.175201757627;
    pCOM[3] << -0.012825433376, -0.007275670525, 0.171431348038;
    pCOM[4] << -0.019870246084, -0.045969255056, -0.011506941050;
    pCOM[5] << 0.051509407797, 0.002163982128, 0.069388069491;
    mass << 0.826, 1.932, 2.82, 1.809, 1.634, 1.003;

    getLegAngles(side, legAngles);

    for (int i = 0; i < 6; i++) {
        huboLegFK(jointLoc, legAngles, side);
        sumMp += jointLoc * pCOM[i] * mass(i);
        sumMasses += mass(i);
    }

    legCOM = sumMp / sumMasses;
    return legCOM;
    }
*/
