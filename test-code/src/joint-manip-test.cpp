#include <manip.h>

int main(int argc, char** argv)
{

    // Open up ach channel
    ach_channel_t chan_manip_cmd;

    ach_status_t r = ach_open( &chan_manip_cmd, CHAN_HUBO_MANIP_CMD, NULL );
    daemon_assert( r==ACH_OK, __LINE__ );

    // Initialize manip_cmd struct
    hubo_manip_cmd_t manip_cmd;
    memset( &(manip_cmd), 0, sizeof(hubo_manip_cmd_t) );

    // Set components of manip_cmd struct for straight joint angles use
    for(int side=0; side<2; side++)
    {
        manip_cmd.m_mode[side] = MC_ANGLES;
        manip_cmd.m_ctrl[side] = MC_NONE;
        manip_cmd.m_grasp[side] = MC_GRASP_LIMP;
        manip_cmd.interrupt[side] = true;
    }

    // Set static variables for manip_cmd
    manip_cmd.waistAngle = 0.0;
    manip_cmd.stopNorm = 1e-10;
    manip_cmd.convergeNorm = 0.0075;

    // Set joint angles for both arms
    fprintf(stderr, "Arm Joint Count: %d\n", ARM_JOINT_COUNT);
    double ltArmAngles[ARM_JOINT_COUNT] = {-M_PI/2, M_PI/2, 0, -M_PI/2, 0, 0, 0, 0, 0, 0};
    double rtArmAngles[ARM_JOINT_COUNT] = {-M_PI/2, -M_PI/2, 0, -M_PI/2, 0, 0, 0, 0, 0, 0};

    memcpy( &manip_cmd.arm_angles[LEFT], &ltArmAngles, sizeof(ltArmAngles) );
    memcpy( &manip_cmd.arm_angles[RIGHT], &rtArmAngles, sizeof(rtArmAngles) );

    r = ach_put( &chan_manip_cmd, &manip_cmd, sizeof(hubo_manip_cmd_t) );
    fprintf(stderr, "ach_put manip_cmd result: %s\n", ach_result_to_string(r));
    std::cout << "manip cmd\n" << manip_cmd << std::endl;
}
