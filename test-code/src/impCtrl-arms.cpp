#include <Hubo_Control.h>
#include <iostream>
#include <fstream>
#include <getopt.h>
#include "impedanceController.h"
#include <termio.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>


static int tty_unbuffered(int);     
static void tty_atexit(void);
static int tty_reset(int);
static void tweak_init();

static struct termios save_termios;
static int  ttysavefd = -1;

double M, Q, K; // best numbers so far: M=0.008, Q=0.4, K=7.0
double MStep, QStep, KStep;


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

    M = 0.002; // user set inertia (kg)
    Q = 0.1; // sungmoon changed to .1 (on 4/2/2013) original was .4 user set damping (N-s/m)
    K = 2.5; //.5;//sungmoon changed to 1 (on 4/2/2013) original was 7; // user set stiffness (N/m)

    MStep = 0.001;
    QStep = 0.1;
    KStep = 1.0;

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
//    std::cout << "Daemonizing as impedanceCtrl\n";
//    Hubo_Control hubo("impedanceCtrl");

    //=== LOCAL VARIABLES ===//
    Vector6d rActualAngles, rArmAnglesCurrent, rArmAnglesNext, checkr;
    Vector6d lActualAngles, lArmAnglesNext, lArmAnglesCurrent, checkl;
    Vector6d armNomAcc, armNomVel;
    Eigen::Isometry3d lcurrEE, rcurrEE, lTransf, rTransf, lHandCurrent, rHandCurrent, wristTF;
    int i=0, imax=40;
    double dt, ptime;
    double lebDesired, lsyDesired, rebDesired, rsyDesired, dMx, dMy;
    Eigen::Vector2d dqLeb(0,0);
    Eigen::Vector2d dqLsy(0,0);
    Eigen::Vector2d dqReb(0,0);
    Eigen::Vector2d dqRsy(0,0);
    double a=3.0, v=3.0;
    double aMax=15.0, vMax=15.0;
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
    }

    // Define arm nominal acceleration 
    armNomAcc << a, a, a, a*5/3, a, a;
    armNomVel << v, v, v, v*5/3, v, v;

    // Set arm nominal accelerations 
    hubo.setLeftArmNomAcc(armNomAcc);
    hubo.setRightArmNomAcc(armNomAcc);
    hubo.setLeftArmNomSpeeds(armNomVel);
    hubo.setRightArmNomSpeeds(armNomVel);

    usleep(500000);
    //double MdLx = hubo.getLeftHandMx();
    double MdLx = hubo.getLeftHandMy();
    //double MdLy = hubo.getLeftHandMy();
    double MdLy = -hubo.getLeftHandMx();
    double MdRx = hubo.getRightHandMx();
    double MdRy = hubo.getRightHandMy();
    // double FdRz = hubo.getRightHandFz(); // added by sungmoon 4/2/2013 --> RightHandFz not defined in Hubo-Control
    Eigen::Vector3d qNew(0.0, 0.0, 0.0);

    // get current joint angles
    lebDesired = hubo.getJointAngle(LEB);
    rebDesired = hubo.getJointAngle(REB);
    lsyDesired = hubo.getJointAngle(LSY);
    rsyDesired = hubo.getJointAngle(RSY);
    ptime = hubo.getTime(); // set initial time for dt(0)

    std::cout << "Executing control loop ...\n";

    tweak_init();

    char c;

    while(!daemon_sig_quit)
    {
        // get latest state info for Hubo
        hubo.update();

        dt = hubo.getTime() - ptime;
        ptime = hubo.getTime();

        // if new data is available...
        if(dt > 0)
        {
            if ( read(STDIN_FILENO, &c, 1) == 1) {
                switch (c) {
                    case 'u':
                        M += MStep;
                        break;
                    case 'j':
                        if ( M - MStep > 0)
                            M -= MStep;
                        if (M < 0.001)
                            M = 0.001;
                        break;
                    case 'i':
                        Q += QStep;
                        break;
                    case 'k':
                        if ( Q - QStep > 0)
                            Q -= QStep;
                        break;
                    case 'o':
                        K += KStep;
                        break;
                    case 'l':
                        if ( K - KStep > 0)
                            K -= KStep;
                        break;
                }
                std::cout << "M: " << M  << "   " << "Q: " << Q << "   " << "K: " << K << std::endl; 
            }

            // set and get joint angles
            if( left==true )
            {
                // Y-Moment (Elbow)
                qNew.setZero();
                dMy = -hubo.getLeftHandMx() - MdLy;
                qNew = impedanceController(dqLeb, dMy, lebDesired, dt);
                hubo.setJointAngle( LEB, qNew(0), false );
               // hubo.setJointNominalSpeed(LEB*v, qNew(1));
                //hubo.setJointNominalAcceleration(LEB*a, qNew(2));
                // X-Moment (Shoulder Yaw)
                qNew.setZero();
                dMx = hubo.getLeftHandMy() - MdLx;
                qNew = impedanceController(dqLsy, dMx, lsyDesired, dt);
                hubo.setJointAngle( LSY, qNew(0), false );
                //hubo.setJointNominalSpeed(LSY*v, qNew(1));
                //hubo.setJointNominalAcceleration(LSY*a, qNew(2));
            }

            if( right==true )
            {
                // Y-Moment (elbow)
                qNew.setZero();
                dMy = hubo.getRightHandMy() - MdRy;
                qNew = impedanceController(dqReb, dMy, rebDesired, dt);
                hubo.setJointAngle( REB, qNew(0), false );
                // X-Moment (Shoulder yaw)
                qNew.setZero();
                dMx = hubo.getRightHandMx() - MdRx;
                qNew = impedanceController(dqRsy, dMx, rsyDesired, dt);
                hubo.setJointAngle( RSY, qNew(0), false);
                a = fabs((hubo.getJointAngle(REB) - hubo.getJointAngleState(REB)));
                a = (a < 0.2) ? a : 0.2;
                a = (25*a/0.2 > 15) ? 25*a/0.2 : 15;
                hubo.setJointNominalSpeed(REB, a);
                hubo.setJointNominalAcceleration(REB, a);
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
                          << "LEB-Desired: " << lebDesired*180/M_PI
                          << "\nLEB-Actual : " << hubo.getJointAngleState(LEB)*180/M_PI
                          << "\nLeft  hand torques(N-m)(MdX,Mx|MdY,My): " << MdLx << ", " << hubo.getLeftHandMx() << " | " 
                                                                          << MdLy << ", " << hubo.getLeftHandMy()
                          << "\nREB-Desired: " << rebDesired*180/M_PI
                          << "\nREB-qNew   : " << qNew*180/M_PI
                          << "\nREB-Actual : " << hubo.getJointAngleState(REB)*180/M_PI
                          << "\nRight hand torques(N-m)(MdX,Mx|MdY,My): " << MdRx << ", " << hubo.getRightHandMx() << " | " 
                                                                          << MdRy << ", " << hubo.getRightHandMy() //<< " | "
                          //<< " Fz          : " << hubo.getRightHandFz()
                          << "\nM: " << M << "\tQ: " << Q << "\tK: " << K
                          << "\na: " << a
                          << std::endl;
            }
            if(i>=imax) i=0; i++;
        }
    }
    return 0;
}


static int
tty_unbuffered(int fd)      /* put terminal into a raw mode */
{
    struct termios  buf;

    if (tcgetattr(fd, &buf) < 0)
        return(-1);
        
    save_termios = buf; /* structure copy */

    /* echo off, canonical mode off */
    buf.c_lflag &= ~(ECHO | ICANON);

    /* 1 byte at a time, no timer */
    buf.c_cc[VMIN] = 1;
    buf.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSAFLUSH, &buf) < 0)
        return(-1);

    ttysavefd = fd;
    return(0);
}

static int
tty_reset(int fd)       /* restore terminal's mode */
{
    if (tcsetattr(fd, TCSAFLUSH, &save_termios) < 0)
        return(-1);
    return(0);
}

static void
tty_atexit(void)        /* can be set up by atexit(tty_atexit) */
{
    if (ttysavefd >= 0)
        tty_reset(ttysavefd);
}

static void
tweak_init()
{
   /* make stdin unbuffered */
    if (tty_unbuffered(STDIN_FILENO) < 0) {
        std::cout << "Set tty unbuffered error" << std::endl;
        exit(1);
    }

    atexit(tty_atexit);

    /* nonblock I/O */
    int flags;
    if ( (flags = fcntl(STDIN_FILENO, F_GETFL, 0)) == 1) {
        perror("fcntl get flag error");
        exit(1);
    }
    if (fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK) == -1) {
        perror("fcntl set flag error");
        exit(1);
    }
}

