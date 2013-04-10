#ifndef IMPEDANCE_CONTROLLER_H
#define IMPEDANCE_CONTROLLER_H

#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

extern double M, Q, K; // best numbers so far: M=0.008, Q=0.4, K=7.0
extern double MStep, QStep, KStep;


/**
 * @function: impedanceEq( Eigen::Vector2d x )
 * @brief: takes in position and velocity of joints and
 *      returns impedance controller equation for dx
 * @return: 2d vector of qdot and qddot
*/
Eigen::Vector2d impedanceEq(Eigen::Vector2d dq, double dM)
{

    // state space rep.
    Eigen::Vector2d dqDot;
    Eigen::Matrix2d A;
    Eigen::Vector2d B;

    A <<
          0,    1,
        -K/M, -Q/M;

    B << 0, dM/M;

    dqDot = A*dq + B;
    //std::cout << "A:\n" << A << "\nB:\n" << B << "\ndq: " << dq << "\ndqDot: " << dqDot << "\ndM: " << dM << std::endl;
    return dqDot;
};

/**
 * @function: rk4( Hubo_Control &hubo, Eigen::Vector2d dq, double dt )
 * @brief: intergration using runge-kutta method rk4
 *      to get q (q,dq) from dq (dq,ddq).
 * @return: returns the delta_q (joint angle q and joint
 *          velocity dq 
*/
double rk4(Eigen::Vector2d &dq, double dM, double dt )
{
    Eigen::Vector2d k1(0,0), k2(0,0), k3(0,0), k4(0,0); // runge-kutta  increments

    // compute runge-kutta increments
    k1 = impedanceEq(dq, dM);
    k2 = impedanceEq(dq + .5*dt*k1, dM);
    k3 = impedanceEq(dq + .5*dt*k2, dM);
    k4 = impedanceEq(dq + dt*k3, dM);

    // compute new delta q
    dq = dq + dt*(k1 + 2*k2 + 2*k3 + k4)/6;
    return k1(1);
};

/**
 * @function: impedanceController(Hubo_Control &hubo, zmp_traj_element_t &elemCurr, zmp_traj_element_t &elemPrev, double dt)
 * @brief: takes the current desired joint angles and velocities in the legs
 *         and adjusts it to achieve desired moment about the ankles
 * @return: no return value. it adjust the desired joint angles by reference
*/
Eigen::Vector3d impedanceController(Eigen::Vector2d &dq, double dM, double angle, double dt)
{
    Eigen::Vector3d qNew;

    double qDdot = rk4(dq, dM, dt); // pass in dq(n) and get out dq(n+1)
    qNew(0) = angle + dq(0); // qd(n+1) + (qNew(n+1) - qd(n+1))
    qNew(1) = dq(1);
    qNew(2) = qDdot;
    return qNew;
};

#endif // IMPEDANCE_CONTROLLER_H
