/**
 * @file bodyCOM_test.cpp
 * @brief 
 */

//#include "hubo_plus.h"
#include <iostream>

#include "bodyCOM.h"

const bool debug = true;


/**
 * @function main
 */ 
int main(int argc, char **argv)
{
  initLocalCOMs();
  //  hubo_plus hubo; /** Create hubo object */

  int i=0, imax=20;
  //Eigen::Vector6d angles;
  
  //hubo.setJointAngle(RKN, .5);
  //hubo.sendControls();
  
  while(true) {
    /*
   hubo.update();
    usleep(100000);
    hubo.getLeftLegAngleStates(angles);

    legCOM = hubo.getCOMOfLeg(RIGHT);
    std::cout << "COM: " << legCOM.transpose() 
	      << "Angles: " << angles.transpose()
	      << std::endl;
    */
  }
  
  return 0;
}

/**
 * @function initLocalCOMs
 */
void initLocalCOMs() {

  // Leg
  gLocalCOMs_Leg[0] << -0.0347, 0.000, 0.072;
  gLocalCOMs_Leg[1] << 0.049747912596, 0.012531116599, 0.015643664572;
  gLocalCOMs_Leg[2] << -0.019504891350, -0.059577480789, 0.175201757627;
  gLocalCOMs_Leg[3] << -0.012825433376, -0.007275670525, 0.171431348038;
  gLocalCOMs_Leg[4] << -0.019870246084, -0.045969255056, -0.011506941050;
  gLocalCOMs_Leg[5] << 0.051509407797, 0.002163982128, 0.069388069491;

  gMasses_Leg[0] = 0.826;  gMasses_Leg[1] = 1.932;
  gMasses_Leg[2] =  2.82;  gMasses_Leg[3] = 1.809; 
  gMasses_Leg[4] = 1.634;  gMasses_Leg[5] =  1.003;
  
  // Arm
  gLocalCOMs_Arm[0] << -0.0347, 0.000, 0.072;
  gLocalCOMs_Arm[1] << 0.049747912596, 0.012531116599, 0.015643664572;
  gLocalCOMs_Arm[2] << -0.019504891350, -0.059577480789, 0.175201757627;
  gLocalCOMs_Arm[3] << -0.012825433376, -0.007275670525, 0.171431348038;
  gLocalCOMs_Arm[4] << -0.019870246084, -0.045969255056, -0.011506941050;
  gLocalCOMs_Arm[5] << 0.051509407797, 0.002163982128, 0.069388069491;

  gMasses_Arm[0] = 0.826; gMasses_Arm[1] = 1.932; 
  gMasses_Arm[2] = 2.82; gMasses_Arm[3] = 1.809; 
  gMasses_Arm[4] = 1.634; gMasses_Arm[5] = 1.003;

  
}

/**
 * @function getCOM_FUllBody
 */
Eigen::Vector3d getCOM_FullBody( /*hubo_plus _hubo*/ ) {

  Eigen::Vector3d COM_rightArm;
  Eigen::Vector3d COM_rightLeg;
  Eigen::Vector3d COM_leftArm;
  Eigen::Vector3d COM_leftLeg;

  COM_rightArm = getCOM_Arm( 0 );
  COM_leftArm = getCOM_Arm( 1 );
  COM_rightLeg = getCOM_Leg( 0 );
  COM_leftLeg = getCOM_Leg( 1 );

  std::cout<< " * COM Right Arm: "<< COM_rightArm.transpose() << std::endl;
  std::cout<< " * COM Left Arm: "<< COM_leftArm.transpose() << std::endl;
  std::cout<< " * COM Right Leg: "<< COM_rightLeg.transpose() << std::endl;
  std::cout<< " * COM Left Leg: "<< COM_rightLeg.transpose() << std::endl;
}

/**
 * @function getCOM_Arm
 */
Eigen::Vector3d getCOM_Arm( /*hubo_plus _hubo,*/ int _side ) {

  Eigen:: Vector3d COM_arm;
  Eigen::Isometry3d Dofs_arm;
  Eigen::Vector3d sumMassPoses;
  double sumMasses;

  // Initialize mass - posses to 0
  sumMassPoses << 0, 0, 0;
    
  //getArmAngles( _side, armAngles );
    
  for (int i = 0; i < gNumDofs_Arm; i++) {
    //huboArmFK( Dofs_arm, armAngles, _side );
    sumMassPoses += Dofs_arm * gLocalCOMs_Arm[i] *gMasses_Arm[i];
    sumMasses += gMasses_Arm[i];
  }
  
  /** P = Sum(m*p) / Sum(m) */
  COM_arm = sumMassPoses / sumMasses; 
  return COM_arm;

}

/**
 * @function getCOM_Leg
 */
Eigen::Vector3d getCOM_Leg( /*hubo_plus _hubo,*/ int _side ) {

  Eigen:: Vector3d COM_leg;
  Eigen::Isometry3d Dofs_leg;
  Eigen::Vector3d sumMassPoses;
  double sumMasses;

  // Initialize mass - posses to 0
  sumMassPoses << 0, 0, 0;
    
  //getLegAngles( _side, armAngles );
    
  for (int i = 0; i < gNumDofs_Leg; i++) {
    //huboLegFK( Dofs_leg, legAngles, _side );
    sumMassPoses += Dofs_leg * gLocalCOMs_Leg[i] *gMasses_Leg[i];
    sumMasses += gMasses_Leg[i];
  }
  
  /** P = Sum(m*p) / Sum(m) */
  COM_leg = sumMassPoses / sumMasses; 
  return COM_leg;
}

/**
 * @function getCOM_Torso
 */
Eigen::Vector3d getCOM_Torso( /*hubo_plus _hubo,*/ int _side ) {

}


