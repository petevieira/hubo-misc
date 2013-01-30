/** 
 * @file bodyCOM.h
 * @brief Functions to get the COM of the robot
 */

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

/** Constant values */
const int gNumDofs_Arm = 6;
const int gNumDofs_Leg = 6;
const int gNumDofs_Torso = 2;

std::vector<Eigen::Vector3d> gLocalCOMs_Leg( gNumDofs_Leg );
std::vector<double> gMasses_Leg( gNumDofs_Leg );

std::vector<Eigen::Vector3d> gLocalCOMs_Arm( gNumDofs_Arm );
std::vector<double> gMasses_Arm( gNumDofs_Arm );

std::vector<Eigen::Vector3d> gLocalCOMs_Torso( gNumDofs_Torso );


/** Functions */
void initLocalCOMs();
Eigen::Vector3d getCOM_FullBody( /*hubo_plus _hubo*/ );
Eigen::Vector3d getCOM_Arm( /*hubo_plus _hubo,*/ int _side );
Eigen::Vector3d getCOM_Leg( /*hubo_plus _hubo,*/ int _side );
Eigen::Vector3d getCOM_Torso( /*hubo_plus _hubo,*/ int _side );
