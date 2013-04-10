#include <Hubo_Control.h>
#include <iostream>
#include <fstream>
#include <getopt.h>
#include "Collision_Checker.h"
#include "Teleop.h"

static const double FOOT_WIDTH = .130; // Width of Hubo's foot in meters

/**
 * @function: usage(std::ostream& ostr)
 * @brief: Prints out how to run this program
*/
void usage(std::ostream& ostr)
{
    ostr << 
        "USAGE:\n"
        "\n"
        "teleop-legs [OPTIONS] \n"
        "\n"
        "EXAMPLES:\n"
        "\n"
        "./teleop-legs -l -r5           Control the left leg using the default sensor number and right leg using sensor 5.\n"
        "./teleop-legs -l1 -r2          Control both legs using sensor 1 for the left leg and sensor 2 for the right leg.\n"
        "./teleop-legs --left           Control the left leg using the default sensor number.\n"
        "./teleop-legs --right=4        Control the right leg using sensor number 4.\n"
        "./teleop-legs -l -r -dliberty  Control both legs using default sensor numbers and using the \"liberty\" device.\n"
        "\n"
        "OPTIONS:\n"
        "\n"
        "  -lSENSOR_NUMBER, --left=SENSOR_NUMBER    Control left leg using sensor SENSOR_NUMBER (Default is 3).\n"
        "  -rSENSOR_NUMBER, --right=SENSOR_NUMBER   Control right leg using sensor SENSOR_NUMBER (Default is 4).\n"
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
 * Hubo's arm joints based on the poses of the foots
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
    int leftSensorNumberDefault = 3; // default left foot sensor number
    int rightSensorNumberDefault = 4; // default right foot sensor number
    int leftSensorNumber = leftSensorNumberDefault; // left foot sensor number
    int rightSensorNumber = rightSensorNumberDefault; // right foot sensor number
    const char *teleopDeviceName = "liberty"; // name of teleop device

    // local variables
    Vector6d lActualAngles, lLegAnglesNext, lLegAnglesCurrent;
    Vector6d rActualAngles, rLegAnglesNext, rLegAnglesCurrent;
    Vector3d lFootOrigin, lSensorChange, lSensorOrigin, lSensorPos;
    Vector3d rFootOrigin, rSensorChange, rSensorOrigin, rSensorPos; 
    Eigen::Matrix3d lRotInitial, rRotInitial, lSensorRot, rSensorRot;
    Eigen::Isometry3d lFootInitialPose, lFootPoseCurrent, lFootPoseDesired;
    Eigen::Isometry3d rFootInitialPose, rFootPoseCurrent, rFootPoseDesired;
    Vector6d speeds; speeds << 0.75, 0.75, 0.75, 0.75, 0.75, 0.75;
    Vector6d accels; accels << 0.40, 0.40, 0.40, 0.40, 0.40, 0.40;
    double initialFootHeight = 0.1;
    double dt, ptime;
    int counter=0, counterMax=50;
    bool updateRight;

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

    // make sure the sensor numbers are not the same for both feet
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
    Teleop teleop(teleopDeviceName); // Create Teleop object

    if (left == true) // if using the left arm
    {
        teleop.getPose( lSensorOrigin, lRotInitial, leftSensorNumber, true ); // get initial sensor pose
        hubo.setLeftLegNomSpeeds( speeds ); // Set left arm nominal joint speeds
        hubo.setLeftLegNomAcc( accels ); // Set left arm nominal joint accelerations
    }

    if (right == true) // if using the right arm
    {
        if(left == true) updateRight = false; else updateRight = true;
        teleop.getPose( rSensorOrigin, rRotInitial, rightSensorNumber, updateRight ); // get initial sensor pose
        hubo.setRightLegNomSpeeds( speeds ); // Set right arm nominal joint speeds
        hubo.setRightLegNomAcc( accels ); // Set right arm nomimal joint accelerations
    }

    if(send == true) // if user wants to send commands
        hubo.sendControls(); // send commands to the control daemon

    if(left == true)
    {
        hubo.getLeftLegAngles(lLegAnglesNext);
        hubo.huboLegFK(lFootInitialPose, lLegAnglesNext, LEFT); // Get left foot pose
        lFootOrigin = lFootInitialPose.translation(); // Set relative zero for foot location
    }

    if(right == true)
    {
        hubo.getRightLegAngles(rLegAnglesNext);
        hubo.huboLegFK(rFootInitialPose, rLegAnglesNext, RIGHT); // Get right foot pose
        rFootOrigin = rFootInitialPose.translation(); // Set relative zero for foot location
    }

    // while the daemon is running
    while(!daemon_sig_quit)
    {
        hubo.update(); // Get latest state info from Hubo

        dt = hubo.getTime() - ptime; // compute change in time
        ptime = hubo.getTime(); // get current time

        if(dt>0 || (send == false && print == true)); // if new data was received over ach
        {
            if(left == true) // if using left arm
            {
                hubo.getLeftLegAngles(lLegAnglesCurrent); // get left arm joint angles
                hubo.huboLegFK(lFootPoseCurrent, lLegAnglesCurrent, LEFT); // get left foot pose
                teleop.getPose(lSensorPos, lSensorRot, leftSensorNumber, true); // get teleop data
                lSensorChange = lSensorPos - lSensorOrigin; // compute teleop relative translation
                lFootPoseDesired = Eigen::Matrix4d::Identity(); // create 4d identity matrix
                lFootPoseDesired.translate(lSensorChange + lFootOrigin); // pretranslate relative translation
                // make sure feet don't cross sagittal plane
                if(lFootPoseDesired(1,3) - FOOT_WIDTH/2 < 0)
                    lFootPoseDesired(1,3) = FOOT_WIDTH/2;

                lFootPoseDesired.rotate(lSensorRot); // add rotation to top-left of TF matrix
                hubo.huboLegIK( lLegAnglesNext, lFootPoseDesired, lLegAnglesCurrent, LEFT ); // get joint angles for desired TF
                hubo.setLeftLegAngles( lLegAnglesNext, false ); // set joint angles
                hubo.getLeftLegAngles( lActualAngles ); // get current joint angles
            }

            if( right==true ) // if using right arm
            {
                if(left == true) updateRight = false; else updateRight = true;
                hubo.getRightLegAngles(rLegAnglesCurrent); // get right arm joint angles
                hubo.huboLegFK(rFootPoseCurrent, rLegAnglesCurrent, RIGHT); // get right foot pose
                teleop.getPose(rSensorPos, rSensorRot, rightSensorNumber, updateRight); // get teleop data
                rSensorChange = rSensorPos - rSensorOrigin; // compute teleop relative translation
                rFootPoseDesired = Eigen::Matrix4d::Identity(); // create 4d identity matrix
                rFootPoseDesired.translate(rSensorChange + rFootOrigin); // pretranslation by relative translation
                // make sure feet don't cross sagittal plane
                if(rFootPoseDesired(1,3) + FOOT_WIDTH/2 > 0)
                    rFootPoseDesired(1,3) = -FOOT_WIDTH/2;
       
                rFootPoseDesired.rotate(rSensorRot); // add rotation to top-left corner of TF matrix
                hubo.huboLegIK( rLegAnglesNext, rFootPoseDesired, rLegAnglesCurrent, RIGHT ); // get joint angles for desired TF
                hubo.setRightLegAngles( rLegAnglesNext, false ); // set joint angles
                hubo.getRightLegAngles( rActualAngles ); // get current joint angles
            }

            if( send == true ) // if user wants to send commands the control boards
                hubo.sendControls(); // send reference commands set above

            if( counter>=counterMax && print==true ) // if user wants output, print output every imax cycles
            {
                std::cout
                          << "Teleop Position Lt(m): " << lSensorChange.transpose()
                          << "\nTeleop Rotation Lt: \n" << lSensorRot
                          << "\nTeleop Position Rt(m): " << rSensorChange.transpose()
                          << "\nTeleop Rotation Rt: \n" << rSensorRot
                          << "\nLeft  Leg Actual Angles (rad): " << lActualAngles.transpose()
                          << "\nLeft  Leg Desired Angles(rad): " << lLegAnglesNext.transpose()
                          << "\nRight Leg Actual Angles (rad): " << rActualAngles.transpose()
                          << "\nRight Leg Desired Angles(rad): " << rLegAnglesNext.transpose()
                          << "\nRight Foot Desired Pose: \n" << rFootPoseDesired.matrix()
                          << "\nLeft Foot Desired Pose: \n" << lFootPoseDesired.matrix()
                          << "\nRight foot torques(N-m)(Mx,My): " << hubo.getRightFootMx() << ", " << hubo.getRightFootMy()
                          << "\nLeft  foot torques(N-m)(Mx,My): " << hubo.getLeftFootMx() << ", " << hubo.getLeftFootMy()
                          << std::endl;
            }
            if(counter>=counterMax) counter=0; counter++; // reset counter if it reaches counterMax
        }
    }
}
