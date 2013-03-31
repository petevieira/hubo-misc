#include <Hubo_Control.h>
#include <iostream>
#include <fstream>
#include <getopt.h>

/**
 * @function: impedanceEq( Eigen::Vector2d x )
 * @brief: takes in position and velocity of joints and
 *      returns impedance controller equation for dx
 * @return: 2d vector of qdot and qddot
*/
Eigen::Vector2d impedanceEq(Eigen::Vector2d dq, double dM)
{
    double M, Q, K;
    M = 1; // user set inertia
    Q = 100; // user set damping
    K = 100; // user set stiffness

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
}

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
}

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
}

/**
 * Prints out how to run this program
*/
void usage(std::ostream& ostr) {
    ostr << 
        "usage: fastrak-arms [OPTIONS] \n"
        "\n"
        "OPTIONS:\n"
        "\n"
        "  -l, --left           Control left arm only.\n"
        "  -r, --right          Control right arm only.\n"
        "  -n, --nosend         Don't send commands to Hubo.\n"
        "  -V, --verbose        Show output.\n"
        "  -H, --help           See this message\n";
}

int main(int argc, char **argv)
{
    // check if no arguments given, if not report usage
    if (argc < 2)
    {
        usage(std::cerr);
        return 1;
    }

    bool print = false; // whether to print output or not
    bool left = false; // whether to set left arm angles
    bool right = false; // whether to set right arm angles
    bool send = true; // whether to send commands or not

    // command line lone options
    const struct option long_options[] = 
    {
        { "left",       no_argument, 0, 'l' },
        { "right",      no_argument, 0, 'r' },
        { "nosend",     no_argument, 0, 'n' },
        { "verbose",    no_argument, 0, 'V' },
        { "help",       no_argument, 0, 'H' },
        { 0,            0,           0,  0  },
    };

    // command line short options
    const char* short_options = "lrnVH";

    // command line option and option index number
    int opt, option_index;

    // loop through command line options and set values accordingly
    while ( (opt = getopt_long(argc, argv, short_options, long_options, &option_index)) != -1 )
    {
        switch (opt)
        {
            case 'l': left = true; break;
            case 'r': right = true; break;
            case 'n': send = false; break;
            case 'V': print = true; break;
            case 'H': usage(std::cout); exit(0); break;
            default:  usage(std::cerr); exit(1); break;
        }
    }

    //=== OBJECTS ===//
    // Create Hubo_Control object
    Hubo_Control hubo;
//    Hubo_Control hubo("fastrak-arms");

    //=== LOCAL VARIABLES ===//
    Vector6d rActualAngles, rArmAnglesCurrent, rArmAnglesNext, checkr;
    Vector6d lActualAngles, lArmAnglesNext, lArmAnglesCurrent, checkl;
    Vector6d armNomAcc, armNomVel;
    Eigen::Isometry3d lcurrEE, rcurrEE, lTransf, rTransf, lHandCurrent, rHandCurrent, wristTF;
    int i=0, imax=40;
    double dt, ptime;
    double lebDesired, rebDesired, dM;
    Eigen::Vector2d dqL(0,0);
    Eigen::Vector2d dqR(0,0);

    // Define starting joint angles for the arms 
    lArmAnglesNext << 0, 0, 0, 0, 0, 0;
    rArmAnglesNext << 0, 0, 0, 0, 0, 0;
 
    // Set the arm joint angles and send commands to the control daemon
    if(left == true)
    {
        lArmAnglesNext << 0, -.3, 0, -M_PI/2, 0, 0;
        hubo.setLeftArmAngles( lArmAnglesNext, true );
    }
    if(right == true)
    {
        rArmAnglesNext << 0, .3, 0, -M_PI/2, 0, 0;
        hubo.setRightArmAngles( rArmAnglesNext, true );
    }
    // While the norm of the right arm angles is greater than 0.075
    // keep waiting for arm to get to desired position
    while ((lArmAnglesNext - checkl).norm() > 0.075 || (rArmAnglesNext - checkr).norm() > 0.075)
    {
        hubo.update(); // Get latest data from ach channels
        hubo.getLeftArmAngles(checkl); // Get current left arm joint angles
        hubo.getRightArmAngles(checkr); // Get current right arm joint angles
        printf("moving\n");
    }

    usleep(1000000);
    double MdL = hubo.getLeftHandMy();
    double MdR = hubo.getRightHandMy();
    double qNew = 0.0;
    // get current joint angles
    lebDesired = hubo.getJointAngleState(LEB);
    rebDesired = M_PI/2; //hubo.getJointAngleState(REB);
    int inputD;

    while(!daemon_sig_quit)
    {
        // get latest state info for Hubo
        hubo.update();

        dt = hubo.getTime() - ptime;
        ptime = hubo.getTime();

        // if new data is available...
        if(dt>0)
        {
            // set and get joint angles
            if( left==true )
            {
                dM = hubo.getLeftHandMy() - MdL;
                qNew = impedanceController(dqL, dM, lebDesired, dt);
                hubo.setJointAngle( LEB, qNew, false );
            }

            if( right==true )
            {
                dM = hubo.getRightHandMy() - MdR;
                qNew = impedanceController(dqR, dM, rebDesired, dt);
                hubo.setJointAngle( REB, qNew, false );
    //            std::cout << "       dqR: " << dqR
      //                    << "\n      dM: " << dM
        //                  << "\nrebDesired: " << rebDesired*180/M_PI
          //                << "\n      dt: " << dt << std::endl;
            }

            // send control references
            if( send == true )
            {
                hubo.sendControls();
            }

            // print output every imax cycles
            if( i>=imax && print==true )
            {
                std::cout //<< "\033[2J"
                          << "LEBAngle: " << lebDesired
                          << "\nLeft  hand torques(N-m)(MdX,Mx|MdY,My): " << MdLx << ", " << hubo.getLeftHandMx() << " | " << MdLy << ", " << hubo.getLeftHandMy()
                          << "\nREBAngle: " << rebDesired
                          << "\nRight hand torques(N-m)(MdX,Mx|MdY,My): " << MdRx << ", " << hubo.getRightHandMx() << " | " << MdRy ", " << hubo.getRightHandMy()
                          << std::endl;
            }
            if(i>=imax) i=0; i++;
        }
    }
    return 0;
}
