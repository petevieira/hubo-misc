#include <Hubo_Control.h>
#include <iostream>
#include <fstream>
#include <getopt.h>
#include "Teleop.h"
#include "Collision_Checker.h"

/**
 * @function: usage(std::ostream& ostr)
 * @brief: Prints out how to run this program
*/
void usage(std::ostream& ostr)
{
    ostr <<
        "USAGE:\n"
        "\n"
        "teleop-arms [OPTIONS] \n"
        "\n"
        "EXAMPLES:\n"
        "\n"
        "./teleop-arms -l -r5           Control the left arm using the default sensor number and right arm using sensor 5.\n"
        "./teleop-arms -l1 -r2          Control both arms using sensor 1 for the left arm and sensor 2 for the right arm.\n"
        "./teleop-arms --left           Control the left arm using the default sensor number.\n"
        "./teleop-arms --right=4        Control the right arm using sensor number 4.\n"
        "./teleop-arms -l -r -dliberty  Control both arms using default sensor numbers and using the \"liberty\" device.\n"
        "\n"
        "OPTIONS:\n"
        "\n"
        "  -lSENSOR_NUMBER, --left=SENSOR_NUMBER    Control left arm using sensor SENSOR_NUMBER (SENSOR_NUMBER is optional).\n"
        "  -rSENSOR_NUMBER, --right=SENSOR_NUMBER   Control right arm using sensor SENSOR_NUMBER (SENSOR_NUMBER is optional).\n"
        "  -n,              --nosend                Don't send commands to Hubo.\n"
        "  -dDEVICE_NAME,   --device=DEVICE_NAME    Sets the teleop device to use (optional)(default is \"liberty\").\n"
        "  -V,              --verbose               Show output.\n"
        "  -H,              --help                  See this message\n";
}

/**
 * @function: getSensorNumber(const char* str)
 * @brief: parses the argument for the sensor number
*/
int getSensorNumber(const char* str)
{
    char* endptr;
    long int i = strtol(str, &endptr, 10);
    if(!endptr || *endptr)
    {
        std::cerr << "Error parsing number on command line.\n\n";
        usage(std::cerr);
        exit(1);
    }
    if(i < 1 || i > 8)
    {
        std::cerr << "Woops! Sensor number must be an integer between 1 and 8, inclusive.\n\n";
        usage(std::cerr);
        exit(1);
    }
    return (int)i;
}

/**
 * @function: getDeviceName(const std::string& s)
 * @brief: parses the argument for the teleop device name
*/
const char* getDeviceName(const char *s)
{
    if(0 == strcmp(s, "liberty"))
        return s;
    if(0 == strcmp(s, "fastrak"))
        return s;
    else
    {
        std::cerr << "Not a valid teleop device name: " << s << "\n\n";
        usage(std::cerr);
        exit(1);
    }
}

/**
 * @function: main(int argc, char **argv)
 * @brief: Main function that loops reading the sensors and commanding
 * Hubo's arm joints based on the poses of the hands
*/
int main(int argc, char **argv)
{
    // check if no arguments given, if not report usage
    if (argc < 2)
    {
        usage(std::cerr);
        return 1;
    }

    // command line argument variables
    bool left = false; // whether to set left arm angles
    bool right = false; // whether to set right arm angles
    bool print = false; // whether to print output or not
    bool send = true; // whether to send commands or not
    int leftSensorNumberDefault = 1; // default left hand sensor number
    int rightSensorNumberDefault = 2; // default right hand sensor number
    int leftSensorNumber = leftSensorNumberDefault; // left hand sensor number
    int rightSensorNumber = rightSensorNumberDefault; // right hand sensor number
    const char *teleopDeviceName = "liberty"; // name of teleop device

    // local variables
    Vector6d rActualAngles, rArmAnglesCurrent, rArmAnglesNext, checkr, armNomAcc, armNomVel, dqLeft, dqRight;
    Vector6d lActualAngles, lArmAnglesNext, lArmAnglesCurrent, checkl;
    Vector3d lArmTrans, ltransEE, lTransInitial, lArmTeleop;
    Vector3d rArmTrans, rtransEE, rTransInitial, rArmTeleop; 
    Eigen::Matrix3d lRotInitial, rRotInitial, lRot, rRot;
    Eigen::Isometry3d lcurrEE, rcurrEE, lTransf, rTransf, lHandCurrent, rHandCurrent;
    Vector6d speeds; speeds << 0.75, 0.75, 0.75, 0.75, 0.75, 0.75;
    Vector6d accels; accels << 0.40, 0.40, 0.40, 0.40, 0.40, 0.40;
    int counter=0, counterMax=40;
    double dt, ptime;

    // command line long options
    const struct option long_options[] = 
    {
        { "left",       optional_argument,  0, 'l' },
        { "right",      optional_argument,  0, 'r' },
        { "nosend",     no_argument,        0, 'n' },
        { "device",     optional_argument,  0, 'd' },
        { "verbose",    no_argument,        0, 'V' },
        { "help",       no_argument,        0, 'H' },
        { 0,            0,                  0,  0  },
    };

    // command line short options
    const char* short_options = "l::r::nd::VH";

    // command line option and option index number
    int opt, option_index;

    // loop through command line options and set values accordingly
    while ( (opt = getopt_long(argc, argv, short_options, long_options, &option_index)) != -1 )
    {
        switch (opt)
        {
            case 'l': left = true; if(NULL != optarg) leftSensorNumber = getSensorNumber(optarg); break;
            case 'r': right = true; if(NULL != optarg) rightSensorNumber = getSensorNumber(optarg); break;
            case 'n': send = false; break;
            case 'd': if(NULL != optarg) teleopDeviceName = getDeviceName(optarg); break;
            case 'V': print = true; break;
            case 'H': usage(std::cout); exit(0); break;
            default:  usage(std::cerr); exit(1); break;
        }
    }

    // check to see if there are any invalid arguments on command line
    if (optind < argc)
    {
        std::cerr << "Error: extra arguments on command line.\n\n";
        usage(std::cerr);
        exit(1);
    }

    // make sure the sensor numbers are not the same for both hands
    if(leftSensorNumber == rightSensorNumber)
    {
        if(left == true && right == true)
        {
            std::cerr << "Error!\nSensor #'s are the same.\n"
                      << "Default sensor #'s are \n\tLEFT:  " << leftSensorNumberDefault
                      << "\n\tRIGHT: " << rightSensorNumberDefault
                      << ".\nPlease choose different sensor numbers.\n\n";
            usage(std::cerr);
            exit(1);
        }
    }

    Hubo_Control hubo; // Create Hubo_Control object
//    Hubo_Control hubo("teleop-arms"); // Create Hubo_Control object and daemonize program

    Collision_Checker collisionChecker; // Create Collision_Checker object

    // Create Teleop object
    Teleop teleop("liberty"); // Create Teleop object

    if (left == true) // if using the left arm
    {
        teleop.getPose( lTransInitial, lRotInitial, leftSensorNumber, true ); // get initial sensor pose
        lArmAnglesNext << 0, -.3, 0, -M_PI/2, 0, 0; // Define left arm intial joint angles
        hubo.setLeftArmAngles( lArmAnglesNext ); // Set the left arm joint angles
        hubo.setLeftArmNomSpeeds( speeds ); // Set left arm nominal joint speeds
        hubo.setLeftArmNomAcc( accels ); // Set left arm nominal joint accelerations
    }

    if (right == true) // if using the right arm
    {
        teleop.getPose( rTransInitial, rRotInitial, rightSensorNumber, false ); // get initial sensor pose
        rArmAnglesNext << 0, .3, 0, -M_PI/2, 0, 0; // Define right arm initial joint angles
        hubo.setRightArmAngles( rArmAnglesNext ); // Set right arm joint angles
        hubo.setRightArmNomSpeeds( speeds ); // Set right arm nominal joint speeds
        hubo.setRightArmNomAcc( accels ); // Set right arm nomimal joint accelerations
    }

    if(send == true)
        hubo.sendControls(); // send commands to the control daemon

    // While the norm of the right arm angles is greater than 0.075
    // keep waiting for arm to get to desired position
    while ((lArmAnglesNext - checkl).norm() > 0.075 && (rArmAnglesNext - checkr).norm() > 0.075)
    {
        hubo.update(); // Get latest data from ach channels
        hubo.getLeftArmAngles(checkl); // Get current left arm joint angles
        hubo.getRightArmAngles(checkr); // Get current right arm joint angles
    }

    if(left == true)
    {
        hubo.huboArmFK(lcurrEE, lArmAnglesNext, LEFT); // Get left hand pose
        ltransEE = lcurrEE.translation(); // Set relative zero for hand location
    }

    if(right == true)
    {
        hubo.huboArmFK(rcurrEE, rArmAnglesNext, RIGHT); // Get right hand pose
        rtransEE = rcurrEE.translation(); // Set relative zero for hand location
    }

    while(!daemon_sig_quit)
    {
        hubo.update(); // Get latest state info from Hubo

        dt = hubo.getTime() - ptime; // compute change in time
        ptime = hubo.getTime(); // get current time

        if(dt>0) // if new data was received over ach
        {
            if(left == true) // if using left arm
            {
                hubo.getLeftArmAngles(lArmAnglesCurrent); // get left arm joint angles
                hubo.huboArmFK(lHandCurrent, lArmAnglesCurrent, LEFT); // get left hand pose
                teleop.getPose(lArmTeleop, lRot, leftSensorNumber, true); // get teleop data
                lArmTrans = lArmTeleop - lTransInitial; // compute teleop relative translation
                lTransf = Eigen::Matrix4d::Identity(); // create 4d identity matrix
                lTransf.translate(lArmTrans + ltransEE); // pretranslate relative translation
                lTransf.rotate(lRot); // add rotation to top-left of TF matrix
                collisionChecker.checkSelfCollision(lTransf); // check for self-collision
                hubo.huboArmIK( lArmAnglesNext, lTransf, lArmAnglesCurrent, LEFT ); // get joint angles for desired TF
                hubo.setLeftArmAngles( lArmAnglesNext, false ); // set joint angles
                hubo.getLeftArmAngles( lActualAngles ); // get current joint angles
            }

            if( right==true ) // if using right arm
                hubo.getRightArmAngles(rArmAnglesCurrent); // get right arm joint angles
                hubo.huboArmFK(rHandCurrent, rArmAnglesCurrent, RIGHT); // get right hand pose
                teleop.getPose(rArmTeleop, rRot, rightSensorNumber, false); // get teleop data
                rArmTrans = rArmTeleop - rTransInitial; // compute teleop relative translation
                rTransf = Eigen::Matrix4d::Identity(); // create 4d identity matrix
                rTransf.translate(rArmTrans + rtransEE); // pretranslation by relative translation
                rTransf.rotate(rRot); // add rotation to top-left corner of TF matrix
                collisionChecker.checkSelfCollision(rTransf); // check for self-collision
                hubo.huboArmIK( rArmAnglesNext, rTransf, rArmAnglesCurrent, RIGHT ); // get joint angles for desired TF
                hubo.setRightArmAngles( rArmAnglesNext, false ); // set joint angles
                hubo.getRightArmAngles( rActualAngles ); // get current joint angles
            }

            if( send == true ) // if user wants to send commands the control boards
                hubo.sendControls(); // send reference commands set above

            if( counter>=counterMax && print==true ) // if user wants output, print output every imax cycles
            {
                std::cout
                          << "Teleop Position Lt(m): " << lArmTrans.transpose()
                          << "\nTeleop Rotation Lt: \n" << lRot
                          << "\nTeleop Position Rt(m): " << rArmTrans.transpose()
                          << "\nTeleop Rotation Rt: \n" << rRot
                          << "\nLeft  Arm Actual Angles (rad): " << lActualAngles.transpose()
                          << "\nLeft  Arm Desired Angles(rad): " << lArmAnglesNext.transpose()
                          << "\nRight Arm Actual Angles (rad): " << rActualAngles.transpose()
                          << "\nRight Arm Desired Angles(rad): " << rArmAnglesNext.transpose()
                          << "\nRight hand torques(N-m)(Mx,My): " << hubo.getRightHandMx() << ", " << hubo.getRightHandMy()
                          << "\nLeft  hand torques(N-m)(Mx,My): " << hubo.getLeftHandMx() << ", " << hubo.getLeftHandMy()
                          << "\ndqLeft : " << dqLeft.transpose()
                          << "\ndqRight: " << dqRight.transpose()
                          << std::endl;
            }
            if(counter>=counterMax) counter=0; counter++; // reset counter if it reaches counterMax
        }
    }
}
