#ifndef IMPEDANCE_CONTROLLER_H
#define IMPEDANCE_CONTROLLER_H

#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
 
/**
 * @function: impedanceEq( Eigen::Vector2d x )
 * @brief: takes in position and velocity of joints and
 *      returns impedance controller equation for dx
 * @return: 2d vector of qdot and qddot
*/
Eigen::Vector2d impedanceEq(Eigen::Vector2d dq, double dM)
{
    double M, Q, K; // best numbers so far: M=0.008, Q=0.4, K=7.0
    M = .008; // user set inertia (kg)
    Q = 1.5; // sungmoon changed to .1 (on 4/2/2013) original was .4 user set damping (N-s/m)
    K = .5; //.5;//sungmoon changed to 1 (on 4/2/2013) original was 7; // user set stiffness (N/m)

    // state space rep.
    Eigen::Vector2d dqDot;
    Eigen::Matrix2d A;
    Eigen::Vector2d B;

    A <<
          0,    1,
        -K/M, -Q/M;

    B << 0, dM/M;

    dqDot = A*dq + B;

    return dqDot;
};

/**
 * @function: rk4( Hubo_Control &hubo, Eigen::Vector2d dq, double dt )
 * @brief: intergration using runge-kutta method rk4
 *      to get q (q,dq) from dq (dq,ddq).
 * @return: returns the delta_q (joint angle q and joint
 *          velocity dq 
*/
void rk4(Eigen::Vector2d &dq, double dM, double dt )
{
    Eigen::Vector2d k1(0,0), k2(0,0), k3(0,0), k4(0,0); // runge-kutta  increments

    // compute runge-kutta increments
    k1 = impedanceEq(dq, dM);
    k2 = impedanceEq(dq + .5*dt*k1, dM);
    k3 = impedanceEq(dq + .5*dt*k2, dM);
    k4 = impedanceEq(dq + dt*k3, dM);

    // compute new delta q
    dq = dq + dt*(k1 + 2*k2 + 2*k3 + k4)/6;
};

/**
 * @function: impedanceController(Hubo_Control &hubo, zmp_traj_element_t &elemCurr, zmp_traj_element_t &elemPrev, double dt)
 * @brief: takes the current desired joint angles and velocities in the legs
 *         and adjusts it to achieve desired moment about the ankles
 * @return: no return value. it adjust the desired joint angles by reference
*/
double impedanceController(Eigen::Vector2d &dq, double dM, double angle, double dt)
{
    double qNew;

    rk4(dq, dM, dt); // pass in dq(n) and get out dq(n+1)
    qNew = angle + dq(0); // qd(n+1) + (qNew(n+1) - qd(n+1))
    return qNew;
};

#endif // IMPEDANCE_CONTROLLER_H
