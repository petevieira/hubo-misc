#include <Hubo_Tech.h>

void balance(Hubo_Tech &hubo);
void shiftToSide(int side);
void shiftToSide2(int side);
void setDoubleSupportLimits(Hubo_Tech &hubo);
Vector3d getBalancePt2COM(Hubo_Tech &hubo, Vector3d Neck2COM);
void pitchAnkles(Hubo_Tech &hubo);

int main(int argc, char **argv)
{
    // Create Hubo_Tech object
    Hubo_Tech hubo;
    hubo.loadURDFModel("/home/pete/Downloads/hubo-motion-rt/src/dart_Lite/urdf/huboplus.urdf");

//    balance(hubo);
//    shiftToSide(right);
//    shiftToSide2(RIGHT);
    pitchAnkles(hubo);
}

void pitchAnkles(Hubo_Tech &hubo)
{

    double velLAP=0.001, velLAR, velLHR;
    double velRAP, velRAR, velRHR;
    Eigen::Isometry3d ltFootTF, ltFT2Neck;
    Vector6d ltLegAngles;
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

            hubo.setJointVelocity(LAP, velLAP, true);

            hubo.getLegAngles(LEFT, ltLegAngles);
            hubo.huboLegFK(ltFootTF, ltLegAngles, LEFT);
            ltFT2Neck = ltFootTF.inverse();

            if(i>imax)
            {
                std::cout
                    << "NeckinFootFrame = \n" << ltFT2Neck.matrix()
                    << "\nvelLAP = " << ltLegAngles.transpose()
                << std::endl;
                i = 0;
            }
        }
    }
}
    


void balance(Hubo_Tech &hubo)
{
    // LOCAL VARIABLES
    Eigen::Vector3d Neck2COM, balPt2COM;
    double velLAP, velLAR, velLHR;
    double velRAP, velRAR, velRHR;
    double compGainAnkleRoll = 0.001;
    double compGainAnklePitch = 0.001;
    double KpCOM = 0.1;
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

            // reset hip roll limits

            setDoubleSupportLimits(hubo);
            // get center of mass vector for Hubo w.r.t. Neck
            Neck2COM = hubo.getCOM_FullBody();

//            balPt2COM = getBalancePt2COM(hubo, Neck2COM);

            // Compute ankle joint velocities using resistant(against COM) 
            // and compliant(with ankle moments) terms
            velLAP = (KpCOM * balPt2COM(0)) + compGainAnklePitch*hubo.getLeftFootMy();
            velLAR = -(KpCOM * balPt2COM(1)) + compGainAnkleRoll*hubo.getLeftFootMx();
            velRAP = (KpCOM * balPt2COM(0)) + compGainAnklePitch*hubo.getRightFootMy();
            velRAR = -(KpCOM * balPt2COM(1)) + compGainAnkleRoll*hubo.getRightFootMx();

            // Compute hip joint velocities
            velLHR = -velLAR;    ///set LHR velocity opposite to LAR velocity
            velRHR = -velRAR;    ///set RHR velocity opposite to RAR velocity

            hubo.setJointVelocity(LAP, velLAP);
            hubo.setJointVelocity(LAR, velLAR);
            hubo.setJointVelocity(RAP, velRAP);
            hubo.setJointVelocity(RAR, velRAR);
            hubo.setJointVelocity(LHR, velLHR);
            hubo.setJointVelocity(RHR, velRHR);

            // Send commands to control-daemon
            hubo.sendControls();
 
            // Print out data
            if(i >= imax)
            {
                std::cout
                    << "COM: " << balPt2COM.transpose()
                    << "rAP,AR,HR: " << velLAP << ", " << velLAR << ", " << velLHR
                    << "lAP,AR,HR: " << velRAP << ", " << velRAR << ", " << velRHR
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
    Eigen::Vector3d Neck2COM;
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
            Neck2COM = hubo.getCOM_FullBody(); ///get center of mass vector for hubo

            if(side == LEFT) // lean left (ie, move feet right)
            {
                rtFootTF(1,3) += Neck2COM(1);
                ltFootTF(1,3) = Neck2COM(1) - ltFootTF(1,3);
            }
            else if(side == RIGHT) // lean right (ie, move feet left)
            {
                rtFootTF(1,3) = Neck2COM(1) - rtFootTF(1,3);
                ltFootTF(1,3) += Neck2COM(0);
            }
            else
                std::cout << "You didn't use a valid 'side' argument. Valid arguments are LEFT or RIGHT\n";

            hubo.huboLegIK(rtLegAngles, rtFootTF, rtLegAnglesPrev, RIGHT);
            hubo.huboLegIK(ltLegAngles, ltFootTF, ltLegAnglesPrev, LEFT);

            // Compute ankle joint velocities using resistant and compliant terms
            velLAP = (KpCOM * Neck2COM(0)) - compGainAnklePitch * hubo.getLeftFootMy();
            velLAR = (KpCOM * (ltLegAngles(5) - ltLegAnglesPrev(5))) - compGainAnkleRoll * hubo.getLeftFootMx();
            velRAP = (KpCOM * Neck2COM(0)) - compGainAnklePitch * hubo.getRightFootMy();
            velRAR = (KpCOM * (rtLegAngles(5) - rtLegAnglesPrev(5))) - compGainAnkleRoll * hubo.getRightFootMx();

            // Compute hip joint velocities
            velLHR = -velLAR;    ///set LHR velocity opposite to LAR velocity
            velRHR = -velRAR;    ///set RHR velocity opposite to RAR velocity

            hubo.setJointVelocity(LAP, velLAP);
            hubo.setJointVelocity(LAR, velLAR);
            hubo.setJointVelocity(RAP, velRAP);
            hubo.setJointVelocity(RAR, velRAR);
            hubo.setJointVelocity(LHR, velLHR);
            hubo.setJointVelocity(RHR, velRHR);

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
    Eigen::Vector3d Neck2COM;
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

            hubo.setJointVelocity(LAP, velLAP);
            hubo.setJointVelocity(LAR, velLAR);
            hubo.setJointVelocity(RAP, velRAP);
            hubo.setJointVelocity(RAR, velRAR);
            hubo.setJointVelocity(LHR, velLHR);
            hubo.setJointVelocity(RHR, velRHR);

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

void setDoubleSupportLimits(Hubo_Tech &hubo)
{
    double LHRcurrentMin = hubo.getJointAngleMin(LHR);
    double RHRcurrentMax = hubo.getJointAngleMax(RHR);
    double LHRAngle = hubo.getJointAngle(LHR);
    double RHRAngle = hubo.getJointAngle(RHR);
    hubo.setJointAngleMin(LHR, (RHRAngle < LHRcurrentMin ? RHRAngle : LHRcurrentMin));
    hubo.setJointAngleMax(RHR, (LHRAngle > RHRcurrentMax ? LHRAngle : RHRcurrentMax));
}

Vector3d getBalancePt2COM(Hubo_Tech &hubo, Vector3d Neck2COMVector)
{
    Vector3d balPt2COM;
    Eigen::Isometry3d Neck2BalPt, Neck2COM;
    Vector6d ltLegAngles, rtLegAngles;
    Eigen::Isometry3d ltFootTF, rtFootTF, worldOrigin;
    double ankle2Middle = 0;//.025;

    hubo.getLegAngles(LEFT, ltLegAngles);
    hubo.getLegAngles(RIGHT, rtLegAngles);
    hubo.huboLegFK(ltFootTF, ltLegAngles, LEFT);
    hubo.huboLegFK(rtFootTF, rtLegAngles, RIGHT);

    worldOrigin = Eigen::Matrix4d::Identity();

    worldOrigin.rotate(ltFootTF.rotation());
    worldOrigin(0,3) = ((ltFootTF(0,3) + ankle2Middle) + (rtFootTF(0,3) + ankle2Middle)) / 2;
    worldOrigin(1,3) = (ltFootTF(1,3) + rtFootTF(1,3)) / 2;
    worldOrigin(2,3) = (ltFootTF(2,3) + rtFootTF(2,3)) / 2;

    Neck2COM = Eigen::Matrix4d::Identity();
    Neck2COM.translate(Neck2COMVector); 

    balPt2COM = (Neck2BalPt.inverse() * Neck2COM).translation();
    return balPt2COM;
}
