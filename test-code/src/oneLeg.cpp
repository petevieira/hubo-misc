#include <Hubo_Tech.h>

void balance(Hubo_Tech hubo);
void shiftToSide(int side);
void shiftToSide2(int side);

int main(int argc, char **argv)
{
    // Create Hubo_Tech object
    Hubo_Tech hubo;
    hubo.loadURDFModel("/home/rapierevite/hubo-motion-rt/src/dart_Lite/urdf/huboplus.urdf");

    balance(hubo);
//    shiftToSide(right);
//    shiftToSide2(RIGHT);
}

void balance(Hubo_Tech hubo)
{
    // LOCAL VARIABLES
    Eigen::Vector3d pCOM;
    double velLAP, velLAR, velLHR;
    double velRAP, velRAR, velRHR;
    double compGainAnkleRoll = 0.001;
    double compGainAnklePitch = 0.001;
    double KpCOM = 0.001;
    double dt, prevTime = hubo.getTime();
    double i, imax=50;

    while(!daemon_sig_quit)
    {
        // Get necessary information
        hubo.update();  ///get latest data from ach channels
        // check if new information was received
        dt = hubo.getTime() - prevTime;
        prevTime = hubo.getTime();

        // if new information received
        if(dt > 0)
        {
            i++; // increment i

            // get center of mass vector for Hubo
            pCOM = hubo.getCOM_FullBody();

            // Compute ankle joint velocities using resistant(against COM) 
            // and compliant(with ankle moments) terms
            velLAP = (KpCOM * pCOM(0)) - compGainAnklePitch*hubo.getLeftFootMy();
            velLAR = -(KpCOM * pCOM(1)) - compGainAnkleRoll*hubo.getLeftFootMx();
            velRAP = (KpCOM * pCOM(0)) - compGainAnklePitch*hubo.getRightFootMy();
            velRAR = -(KpCOM * pCOM(1)) - compGainAnkleRoll*hubo.getRightFootMx();

            // Compute hip joint velocities
            velLHR = -velLAR;    ///set LHR velocity opposite to LAR velocity
            velRHR = -velRAR;    ///set RHR velocity opposite to RAR velocity

            // Send commands to control-daemon
            hubo.sendControls();
            
            // Print out data
            if(i >= imax)
            {
                std::cout
                    << "COM: " << pCOM.transpose() 
                << std::endl;
                i = 0;
            }
        }
    }
}

void shiftToSide(int side)
{
    // Create Hubo_Tech object
    Hubo_Tech hubo;

    // LOCAL VARIABLES
    Eigen::Vector3d pCOM;
    Eigen::Isometry3d ltFootTF, rtFootTF;
    Vector6d ltLegAngles, rtLegAngles, ltLegAnglesPrev, rtLegAnglesPrev;
    double velLAP, velLAR, velLHR;
    double velRAP, velRAR, velRHR;
    double compGainAnkleRoll = 0.001;
    double compGainAnklePitch = 0.001;
    double KpCOM = 0.1;
    double dt, prevTime = hubo.getTime();
    double i, imax=50;

    while(!daemon_sig_quit)
    {
        // get latest data from ach channels if any new info
        hubo.update();
        // check if new information was received
        dt = hubo.getTime() - prevTime;
        prevTime = hubo.getTime();

        // if new information received
        if(dt > 0)
        {
            i++; // increment i

            // Get necessary information
            hubo.getLegAngles(RIGHT, rtLegAnglesPrev);   ///get Left leg joint angles
            hubo.getLegAngles(LEFT, ltLegAnglesPrev);   ///get Left leg joint angles
            hubo.huboLegFK(rtFootTF, rtLegAnglesPrev, RIGHT); ///get transformation for 'legSide' foot
            hubo.huboLegFK(ltFootTF, ltLegAnglesPrev, LEFT); ///get transformation for 'legSide' foot
            pCOM = hubo.getCOM_FullBody(); ///get center of mass vector for hubo

            if(side == LEFT) // lean left (ie, move feet right)
            {
                rtFootTF(1,3) += pCOM(1);
                ltFootTF(1,3) = pCOM(1) - ltFootTF(1,3);
            }
            else if(side == RIGHT) // lean right (ie, move feet left)
            {
                rtFootTF(1,3) = pCOM(1) - rtFootTF(1,3);
                ltFootTF(1,3) += pCOM(0);
            }
            else
                std::cout << "You didn't use a valid 'side' argument. Valid arguments are LEFT or RIGHT\n";

            hubo.huboLegIK(rtLegAngles, rtFootTF, rtLegAnglesPrev, RIGHT);
            hubo.huboLegIK(ltLegAngles, ltFootTF, ltLegAnglesPrev, LEFT);

            // Compute ankle joint velocities using resistant and compliant terms
            velLAP = (KpCOM * pCOM(0)) - compGainAnklePitch * hubo.getLeftFootMy();
            velLAR = (KpCOM * (ltLegAngles(5) - ltLegAnglesPrev(5))) - compGainAnkleRoll * hubo.getLeftFootMx();
            velRAP = (KpCOM * pCOM(0)) - compGainAnklePitch * hubo.getRightFootMy();
            velRAR = (KpCOM * (rtLegAngles(5) - rtLegAnglesPrev(5))) - compGainAnkleRoll * hubo.getRightFootMx();

            // Compute hip joint velocities
            velLHR = -velLAR;    ///set LHR velocity opposite to LAR velocity
            velRHR = -velRAR;    ///set RHR velocity opposite to RAR velocity

            // Send commands to control-daemon
            hubo.sendControls();
        }
    }
}

void shiftToSide2(int side)
{
    // Create Hubo_Tech object
    Hubo_Tech hubo;

    // LOCAL VARIABLES
    Eigen::Vector3d pCOM;
    Eigen::Isometry3d ltFootTF, rtFootTF;
    Vector6d ltLegAngles, rtLegAngles, ltLegAnglesPrev, rtLegAnglesPrev;
    double velLAP, velLAR, velLHR;
    double velRAP, velRAR, velRHR;
    double compGainAnkleRoll = 0.001;
    double compGainAnklePitch = 0.001;
    double KpCOM = 0.1;
    double dt, prevTime = hubo.getTime();
    double i, imax=50;

    while(!daemon_sig_quit)
    {
        // get latest data from ach channels if any new info
        hubo.update();
        // check if new information was received
        dt = hubo.getTime() - prevTime;
        prevTime = hubo.getTime();

        // if new information received
        if(dt > 0)
        {
            i++; // increment i

            // Get necessary information
            hubo.getLegAngles(RIGHT, rtLegAnglesPrev);   ///get Left leg joint angles
            hubo.getLegAngles(LEFT, ltLegAnglesPrev);   ///get Left leg joint angles
            hubo.huboLegFK(rtFootTF, rtLegAnglesPrev, RIGHT); ///get transformation for 'legSide' foot
            hubo.huboLegFK(ltFootTF, ltLegAnglesPrev, LEFT); ///get transformation for 'legSide' foot

            if(side == LEFT) // lean left (ie, move feet right)
            {
                ltFootTF(1,3) = 0;
                rtFootTF(1,3) -= ltFootTF(1,3);
            }
            else if(side == RIGHT) // lean right (ie, move feet left)
            {
                rtFootTF(1,3) = 0;
                ltFootTF(1,3) += rtFootTF(1,3);
            }
            else
                std::cout << "You didn't use a valid 'side' argument. Valid arguments are LEFT or RIGHT\n";

            hubo.huboLegIK(rtLegAngles, rtFootTF, rtLegAnglesPrev, RIGHT);
            hubo.huboLegIK(ltLegAngles, ltFootTF, ltLegAnglesPrev, LEFT);

            // Compute ankle joint velocities using resistant and compliant terms
            velLAP = -(KpCOM * ltFootTF(0,3)) - compGainAnklePitch * hubo.getLeftFootMy();
            velLAR = (KpCOM * (ltLegAngles(5) - ltLegAnglesPrev(5))) - compGainAnkleRoll * hubo.getLeftFootMx();
            velRAP = -(KpCOM * ltFootTF(0,3)) - compGainAnklePitch * hubo.getRightFootMy();
            velRAR = (KpCOM * (rtLegAngles(5) - rtLegAnglesPrev(5))) - compGainAnkleRoll * hubo.getRightFootMx();

            // Compute hip joint velocities
            velLHR = -velLAR;    ///set LHR velocity opposite to LAR velocity
            velRHR = -velRAR;    ///set RHR velocity opposite to RAR velocity

            // Send commands to control-daemon
//            hubo.sendControls();
                // Print out data
            if(i >= imax)
            {
                std::cout
                    << "velLAP: " << velLAP
                    << "\nvelLAR: " << velLAR 
                    << "\nvelRAP: " << velRAP
                    << "\nvelRAR: " << velRAR 
                << std::endl;
                i = 0;
            }

        }
    }
}
