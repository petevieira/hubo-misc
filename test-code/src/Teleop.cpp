#include "Teleop.h"

Teleop::Teleop(const char *teleopDeviceName)
{
    initTeleop(teleopDeviceName);
}

Teleop::~Teleop(void)
{
    ach_close(&chan_teleop);
}

flag_t Teleop::initTeleop(const char *teleopDeviceName, bool assert)
{
    if(0 == strcmp(teleopDeviceName,"liberty"))
    {
        TRACKER_NAME = teleopDeviceName;
        NUM_OF_SENSORS = 8;
    }
    else if(0 == strcmp(teleopDeviceName,"fastrak"))
    {
        TRACKER_NAME = teleopDeviceName;
        NUM_OF_SENSORS = 4;
    }
    else
    {
        syslog(LOG_ERR, "Error!. \"%s\" is not a proper tracker name! Please enter \"liberty\" or \"fastrak\".\n", teleopDeviceName);
        exit(EXIT_FAILURE); 
    }

//    teleop.sensorData = new float[NUM_OF_SENSORS][NUM_OF_DATA];
//    std::cout << "sizeof(sensorData): " << sizeof(teleop.sensorData)/sizeof(float) << std::endl;
    
    ach_status_t r = ach_open( &chan_teleop, TRACKER_NAME, NULL );

    if( ACH_OK != r )
    {
        syslog(LOG_ERR, "Unable to open \"%s\" channel: (%d) %s!\n",
            TRACKER_NAME, r, ach_result_to_string((ach_status_t)r));
        return CHAN_OPEN_FAIL;
    }
    syslog(LOG_INFO, "The ach channel \"%s\" opened successfully!\n", TRACKER_NAME);

    teleopScale = 1.0;
    setTeleopScale(1.0);    

    return SUCCESS;    
}

void Teleop::setTeleopScale( double scale ) { teleopScale = scale; }
double Teleop::getTeleopScale() { return teleopScale; };

flag_t Teleop::getPose( Eigen::Vector3d &position, Eigen::Quaterniond &quat, int sensor, bool update )
{
    ach_status_t r = ACH_OK;
    int s;

    if(update) {
        size_t fs;
        r = ach_get( &chan_teleop, teleop.sensorData, sizeof(teleop.sensorData), &fs, NULL, ACH_O_LAST );
        if ( r == ACH_OK ) {
            assert( sizeof(teleop.sensorData) == fs );
        } else {
            syslog(LOG_ERR, "The ach_get() call failed: %s\n", ach_result_to_string(r));
        }
    }

    sensor--;
    
    if ( (sensor >= 0) && (sensor < 8) ) {
        position[0] = teleop.sensorData[sensor][0]/teleopScale;
        position[1] = teleop.sensorData[sensor][1]/teleopScale;
        position[2] = teleop.sensorData[sensor][2]/teleopScale;
        
        quat.w() = (double)teleop.sensorData[sensor][3];
        quat.x() = (double)teleop.sensorData[sensor][4];
        quat.y() = (double)teleop.sensorData[sensor][5];
        quat.z() = (double)teleop.sensorData[sensor][6];
    } else
        return SENSOR_OOB;

    if( ACH_OK != r )
        return TRACKER_STALE;
    return SUCCESS;
}


flag_t Teleop::getPose( Eigen::Vector3d &position, Eigen::Matrix3d &rotation, int sensor, bool update )
{
    Eigen::Quaterniond quat;
    flag_t flag = getPose( position, quat, sensor, update );

    if( flag==SENSOR_OOB )
        return flag;

    rotation = quat.matrix();

    return flag;
}


flag_t Teleop::getPose( Eigen::Isometry3d &tf, int sensor, bool update )
{
    Eigen::Vector3d position;
    Eigen::Quaterniond quat;

    flag_t flag = getPose( position, quat, sensor, update );

    if( flag==SENSOR_OOB )
        return flag;

    tf = Eigen::Matrix4d::Identity();
    tf.translate( position );
    tf.rotate( quat );

    return flag;
}

