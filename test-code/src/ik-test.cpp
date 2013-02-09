#include "hubo_plus.h"
#include <iostream>
#include <ctime>
#include <string>

void ParseSocketCommand(char* cmd, int length, double* y, double* z);

std::string make_daytime_string()
{
  using namespace std; // For time_t, time and ctime;
  time_t now = time(0);
  return ctime(&now);
}

int main()
{

    hubo_plus hubo;
    double xPos, yPos, zPos;
    double y=0;
    Eigen::Isometry3d goal, offset;
    Vector6d q, qprev, qfinal;
    char* request = "<12.34,12.34>";
    size_t request_len = std::strlen(request);
    
    // Set joint angles to initial drill position
    hubo.HuboDrillIK( qfinal, y );
    hubo.setRightArmAngles( qfinal, true );
    
    // Wait till it gets to goal position
    while((qfinal - q).norm() > 0.075)
    {
        // get current right arm joint angles
        hubo.update();
        hubo.getRightArmAngles(q);
    }

    while(true)
    {
        // Convert <x,y> position to doubles
        ParseSocketCommand(request, request_len, &yPos, &zPos); 
        std::cout << "values:" << yPos << "," << zPos;

        // Get current position of right arm and assign it to 'goal'
        hubo.huboArmFK(goal, q, RIGHT);

        // Set goal position y- and z-positions to current position
        // plus the offset from the webcam and keep x-position the same
        goal(1,3) += yPos;
        goal(2,3) += zPos;

        // Get latest state info for Hubo, like joint angles
        hubo.update();
        // Get joint angles for current position
        hubo.getRightArmAngles(qprev);
        // Get joint angles required for new position using IK
        hubo.huboArmIK(q, goal, qprev, RIGHT);
        // Set joint angles and send controls (true)
        hubo.setRightArmAngles(q, true);
    }

    return 0;
}

 // ParseSocketCommand
// Extracts x and y information from our simple socket string

void ParseSocketCommand(char* cmd, int length, double* y, double* z)
{
    int startY, endY, startZ, endZ;
    for(int i = 0; i < length; i++)
    {
        switch (cmd[i])
        {
            case '<':
                startY = i + 1;
                break;
            case ',':
                endY = i - 1;
                startZ = i + 1;
                break;
            case '>':
                endZ = i - 1;
                break;
            default:
                break;
        }
    }
    *y = atof(cmd + startY);
    *z = atof(cmd + startZ);
}




