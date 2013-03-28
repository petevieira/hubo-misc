/**
 * \file Fastrak.h
 * \brief Encapsulates Fastrack access for ACH IPC channels.
 * Currently all methods are static, meaning only one Fastrak, 
 * but we could make the channel name a variable and allow multiple devices.
 *
 * \Author Andrew Price
 */

#ifndef FASTRAK_H
#define FASTRAK_H

#include <Hubo_Control.h>

#define FASTRAK_CHAN_NAME "fastrak"


class Fastrak
{
public:
    double fastrakScale;

    Fastrak();
    ~Fastrak(void);

    /* Function: initFastrak(bool assert=false)
     * Description: Opens Fastrak ach channel
     * Return: Returns a SUCCESS or FAIL flag
    */
    ctrl_flag_t initFastrak(bool assert=false);

    /* Function: setFastrakScale( double scale );
     * Description: Sets the scale of the Fastrak readings
    */
    void setFastrakScale( double scale );

    /* Function: getFastrakScale();
     * Description: Gets the scale of the Fastrak readings
    */
    double getFastrakScale();

    /* Function: getPose( Eigen::Vector3d &position, Eigen::Quaterniond &quat, int sensor=1, bool update=true )
     * Description: Gets the sensor pose as a Eigen Vector3d for the position and a Quaternion of doubles for the orientation
     * Return: Returns a success/fall flag
    */
    ctrl_flag_t getPose( Eigen::Vector3d &position, Eigen::Quaterniond &quat, int sensor=1, bool update=true );

    /* Function: getPose( Eigen::Vector3d &position, Eigen::Matrix3d &rotation, int sensor=1, bool update=true )
     * Description: Gets the sensor pose as a 3x1 vector for the position and a 3x3 matrix of doubles for the orientation
     * Return: Returns a success/fall flag
    */
    ctrl_flag_t getPose( Eigen::Vector3d &position, Eigen::Matrix3d &rotation, int sensor=1, bool update=true );

    /* Function: getPose( Eigen::Isometry3d &tf, int sensor=1, bool update=true )
     * Description: Gets the sensor pose as a isometry3d tranformation matrix
     * Return: Returns a success/fall flag
    */
    ctrl_flag_t getPose( Eigen::Isometry3d &tf, int sensor=1, bool update=true );

    typedef struct
    {
        float data[4][7];
    } fastrak_c_t;

private:
    ach_channel_t chan_fastrak;
    fastrak_c_t fastrak;

};

#endif // FASTRAK_H

