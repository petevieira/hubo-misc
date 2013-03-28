#include "Fastrak.h"


Fastrak::Fastrak()
{
    fastrakScale = 1.0;
}

Fastrak::~Fastrak(void)
{
    //TODO: Do we need to clean up ACH memory?
}


ctrl_flag_t Fastrak::initFastrak(bool assert)
{
    int r = ach_open( &chan_fastrak, FASTRAK_CHAN_NAME, NULL );

    if( ACH_OK != r )
    {
        fprintf(stderr, "Unable to open libery channel: (%d) %s!\n",
            r, ach_result_to_string((ach_status_t)r));
        //if(assert)
        //  daemon_assert( ACH_OK == r, __LINE__ );
        return CHAN_OPEN_FAIL;
    }
    printf("ach open successfully!!!\n");

    return SUCCESS;    
}

void Fastrak::setFastrakScale( double scale ) { fastrakScale = scale; }
double Fastrak::getFastrakScale() { return fastrakScale; };

// Wait until that sensor's data coming in
ctrl_flag_t Fastrak::getPose( Eigen::Vector3d &position, Eigen::Quaterniond &quat, int sensor, bool update )
{
    int r = ACH_OK;
    int s;

    if(update) {
//        printf("in getPose\n");
        size_t fs;
        r = ach_get( &chan_fastrak, fastrak.sData, sizeof(fastrak.sData), &fs, NULL, ACH_O_LAST );
        if ( r == ACH_OK ) {
  //         printf("read %d bytes data\n", fs);
           assert( sizeof(fastrak.sData) == fs );
        } else {
//            return SENSOR_OOB;
 //               printf("ach_get failed\n");
        }
    }

    sensor--;
    
    if ( (sensor >= 0) && (sensor < 8) ) {
        position[0] = fastrak.sData[sensor][0]/fastrakScale;
        position[1] = fastrak.sData[sensor][1]/fastrakScale;
        position[2] = fastrak.sData[sensor][2]/fastrakScale;
        
        quat.w() = (double)fastrak.sData[sensor][3];
        quat.x() = (double)fastrak.sData[sensor][4];
        quat.y() = (double)fastrak.sData[sensor][5];
        quat.z() = (double)fastrak.sData[sensor][6];
    } else
        return SENSOR_OOB;

//    if( ACH_OK != r )
  //      return FASTRAK_STALE;
    return SUCCESS;
}


ctrl_flag_t Fastrak::getPose( Eigen::Vector3d &position, Eigen::Matrix3d &rotation, int sensor, bool update )
{
    Eigen::Quaterniond quat;
    ctrl_flag_t flag = getPose( position, quat, sensor, update );

    if( flag==SENSOR_OOB )
        return flag;

    rotation = quat.matrix();

    return flag;
}


ctrl_flag_t Fastrak::getPose( Eigen::Isometry3d &tf, int sensor, bool update )
{
    Eigen::Vector3d position;
    Eigen::Quaterniond quat;

    ctrl_flag_t flag = getPose( position, quat, sensor, update );

    if( flag==SENSOR_OOB )
        return flag;

    tf = Eigen::Matrix4d::Identity();
    tf.translate( position );
    tf.rotate( quat );

    return flag;
    
}

