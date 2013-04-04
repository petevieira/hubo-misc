#ifndef COLLISION_CHECKER_H
#define COLLISION_CHECKER_H

#include <eigen3/Eigen/Core>

class Collision_Checker
{
public:

    /**
     * Constructor for the Collision_Checking class
    */
    Collision_Checker();

    /**
     * Deconstructor for the Collision_Checking class
    */ 
    ~Collision_Checker();

    /**
     * Checks for collision of the hands with the torso.
     * If the goal location is inside the boundary ellipse defined by parameters a (minor axis)
     * and b (major axis) then the x,y location gets mapped to the surface of the ellipse
    */
    void checkSelfCollision(Eigen::Isometry3d &goal);

private:

    const double xMinor; // minor axis parameter for collision boundary ellipse
    const double yMajor; // major axis parameter for collision boundary ellipse
    const double collisionLimit; // distance to point on ellipse
};

#endif // COLLISION_CHECKER_H
