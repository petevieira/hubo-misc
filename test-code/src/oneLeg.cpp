#include <Hubo_Tech.h>
#include "kinematics/BodyNode.h"

void balance(Hubo_Tech &hubo);
void crouch(Hubo_Tech &hubo);
void shiftToSide(Hubo_Tech &hubo, int side);
void setDoubleSupportLimits(Hubo_Tech &hubo);
Vector3d getBalancePt2COM(Hubo_Tech &hubo, Eigen::Isometry3d world2COM, int phase_t);
Vector3d getBalancePt2COMTorso(Hubo_Tech &hubo, Eigen::Isometry3d world2COM);
void pitchAnkles(Hubo_Tech &hubo);
void checkFKs(Hubo_Tech &hubo);
typedef enum {
    DOUBLE_SUPPORT = 0,
    SINGLE_LEFT,
    SINGLE_RIGHT
} phase_t;

int main(int argc, char **argv)
{
    // Create Hubo_Tech object
    Hubo_Tech hubo;
    //hubo.loadURDFModel("/home/pete/Downloads/hubo-motion-rt/src/dart_Lite/urdf/huboplus.urdf");
    hubo.loadURDFModel("/home/rapierevite/hubo-motion-rt/src/dart_Lite/urdf/huboplus.urdf");

//    balance(hubo);
//    shiftToSide(right);
//    shiftToSide2(RIGHT);
//    pitchAnkles(hubo);
    checkFKs(hubo);
}

void checkFKs(Hubo_Tech &hubo)
{
    Vector6d ltLegAngles, rtLegAngles, lAInit, rAInit, lAInit2, rAInit2;
    Eigen::Isometry3d ltFootTF, rtFootTF, lFInit, rFInit;
    double lap, rap;

    lAInit << 0,0,0,0,0,0;
    rAInit << 0,0,0,0,0,0;
    
    hubo.huboLegFK(lFInit, lAInit, LEFT);
    hubo.huboLegFK(rFInit, rAInit, RIGHT);
    std::cout << "lFInit1: \n" << lFInit.matrix()
            << "\nrFInit1: \n" << rFInit.matrix()
    << std::endl;

    lFInit(2,3) += .05;
    rFInit(2,3) += .05;
    rFInit(0,3) += .10;
    rFInit(1,3) -= .05;

    Eigen::Isometry3d foot;
    double t = 0.3;
    foot(0,0) = cos(t); foot(0,1) =  -sin(t); foot(0,2) = 0; foot(0,3) = 0; 
    foot(1,0) = sin(t); foot(1,1) =  cos(t); foot(1,2) = 0; foot(1,3) = 0; 
    foot(2,0) = 0; foot(2,1) =  0; foot(2,2) = 1; foot(2,3) = 0; 
    foot(3,0) = 0; foot(3,1) =  0; foot(3,2) = 0; foot(3,3) = 1;

    rFInit = rFInit * foot;

    hubo.huboLegIK(lAInit2, lFInit, lAInit, LEFT);
    hubo.huboLegIK(rAInit2, rFInit, rAInit, RIGHT);

    std::cout << "lFInit2: \n" << lFInit.matrix()
            << "\nrFInit2: \n" << rFInit.matrix()
            << "\nlAInit2: " << lAInit2.transpose()
            << "\nrAInit2: " << rAInit2.transpose()
    << std::endl;

    Eigen::Isometry3d centerTF;
    Vector3d center;
    center = (lFInit.translation() + rFInit.translation()) / 2;
    std::cout << "center: " << center.transpose() << std::endl;

    centerTF = Eigen::Matrix4d::Identity();
    centerTF.translate(center);

    std::cout << "centerTF: \n" << centerTF.matrix()
            << "\ncenterTF.inv: \n" << centerTF.inverse().matrix()
    << std::endl;

    while(true)
    {
        std::cout << "LAP: ";
        std::cin >> lap;
        std::cout << "RAP: ";
        std::cin >> rap;
        // set leg angles all to zero
        ltLegAngles << 0,0,0,0,lap,0;
        rtLegAngles << 0,0,0,0,rap,0;

        // get transformation to feet
        hubo.huboLegFK(ltFootTF, ltLegAngles, LEFT);
        hubo.huboLegFK(rtFootTF, rtLegAngles, RIGHT);

        std::cout
            << "Left  Angles: " << ltLegAngles.transpose()
            << "\nRight Angles: " << rtLegAngles.transpose()
            << "\nltFootTF: \n" << ltFootTF.matrix()
            << "\nrtFootTF: \n" << rtFootTF.matrix()
            << "\nltFootTFINV: \n" << ltFootTF.inverse().matrix()
            << "\nrtFootTFINV: \n" << rtFootTF.inverse().matrix()
        << std::endl;
    }
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
    Eigen::Vector3d balPt2COM;
    Vector6d ltLegAngles, rtLegAngles;
    Eigen::Isometry3d world2COM;
    double velLAP, velLAR, velLHR;
    double velRAP, velRAR, velRHR;
    double compGainAnkleRoll = 0.001;
    double compGainAnklePitch = 0.001;
    double KpCOM = 1;
    double dt, prevTime = hubo.getTime();
    double i, imax=50;

    crouch(hubo);

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
            world2COM = hubo.getCOM_FullBody();
//            world2COM = hubo.getCOM_wrtTorso();

            balPt2COM = getBalancePt2COM(hubo, world2COM, DOUBLE_SUPPORT);
//            balPt2COM = getBalancePt2COMTorso(hubo, world2COM);

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
 
            hubo.getLegAngles(LEFT, ltLegAngles);
            hubo.getLegAngles(RIGHT, rtLegAngles);

            // Print out data
            if(i >= imax)
            {
                std::cout
                    << "COM: " << balPt2COM.transpose()
                    << "\nltLegAngles: " << ltLegAngles.transpose()
                    << "\nrlLegAngles: " << rtLegAngles.transpose()
                << std::endl;
                i = 0;
            }
        }
    }
}

void crouch(Hubo_Tech &hubo)
{
    Eigen::Isometry3d ltFootTF, rtFootTF;
    Vector6d ltLegAngles, rtLegAngles, ltLegAnglesPrev, rtLegAnglesPrev;

    // Get necessary information
    hubo.getLegAngles(LEFT, ltLegAnglesPrev);   ///get Left leg joint angles
    hubo.getLegAngles(RIGHT, rtLegAnglesPrev);   ///get Left leg joint angles
    hubo.huboLegFK(ltFootTF, ltLegAnglesPrev, LEFT); ///get transformation for 'legSide' foot
    hubo.huboLegFK(rtFootTF, rtLegAnglesPrev, RIGHT); ///get transformation for 'legSide' foot

    ltFootTF(2,3) += 0.1;
    rtFootTF(2,3) += 0.1;

    hubo.huboLegIK(ltLegAngles, ltFootTF, ltLegAnglesPrev, LEFT);
    hubo.huboLegIK(rtLegAngles, rtFootTF, rtLegAnglesPrev, RIGHT);

    hubo.setLegAngles(LEFT, ltLegAngles);
    hubo.setLegAngles(RIGHT, rtLegAngles);

    hubo.sendControls();
}

void shiftToSide(Hubo_Tech &hubo, int side)
{
    // LOCAL VARIABLES
    Eigen::Isometry3d ltFootTF, rtFootTF, world2COM;
    Vector6d ltLegAngles, rtLegAngles, ltLegAnglesPrev, rtLegAnglesPrev, ltLegVels, rtLegVels;
    Vector3d balPt2COM;
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
            hubo.getLegAngles(LEFT, ltLegAnglesPrev);   ///get Left leg joint angles
            hubo.getLegAngles(RIGHT, rtLegAnglesPrev);   ///get Left leg joint angles
            hubo.huboLegFK(ltFootTF, ltLegAnglesPrev, LEFT); ///get transformation for 'legSide' foot
            hubo.huboLegFK(rtFootTF, rtLegAnglesPrev, RIGHT); ///get transformation for 'legSide' foot
            world2COM = hubo.getCOM_FullBody(); ///get center of mass vector for hubo

            switch(side)
            {
                case LEFT:
                    balPt2COM = getBalancePt2COM(hubo, world2COM, SINGLE_LEFT); break;
                case RIGHT:
                    balPt2COM = getBalancePt2COM(hubo, world2COM, SINGLE_RIGHT); break;
            }

            rtFootTF(0,3) += balPt2COM(0,3);
            rtFootTF(1,3) += balPt2COM(1,3);
            ltFootTF(0,3) += balPt2COM(0,3);
            ltFootTF(1,3) += balPt2COM(1,3);

            hubo.huboLegIK(ltLegAngles, ltFootTF, ltLegAnglesPrev, LEFT);
            hubo.huboLegIK(rtLegAngles, rtFootTF, rtLegAnglesPrev, RIGHT);

            ltLegVels = KpCOM*(ltLegAngles - ltLegAnglesPrev);
            rtLegVels = KpCOM*(rtLegAngles - rtLegAnglesPrev);

            // Compute ankle joint velocities using resistant and compliant terms
            ltLegVels(4) += compGainAnklePitch * hubo.getLeftFootMy(); // add compliance to ankle pitch
            ltLegVels(5) += compGainAnkleRoll * hubo.getLeftFootMx(); // add compliance to ankle roll
            rtLegVels(4) += compGainAnklePitch * hubo.getRightFootMy(); // add compliance to ankle pitch
            rtLegVels(5) += compGainAnkleRoll * hubo.getRightFootMx(); // add compliance to ankle roll

            hubo.setLegVels(LEFT, ltLegVels);
            hubo.setLegVels(RIGHT, rtLegVels);

            // Send commands to control-daemon
            hubo.sendControls();
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

Vector3d getBalancePt2COM(Hubo_Tech &hubo, Eigen::Isometry3d world2COM, int phase_t)
{
    Vector3d balPt2COM;
    Eigen::Isometry3d Neck2BalPt;
    Vector6d ltLegAngles, rtLegAngles;
    Eigen::Isometry3d ltFootTF, rtFootTF, world2CenterOfSP, footOffset;
    double ankle2Middle = .025;

    world2CenterOfSP = Eigen::Matrix4d::Identity();
    footOffset = Eigen::Matrix4d::Identity();
    footOffset(0,3) = ankle2Middle;

    if(phase_t == DOUBLE_SUPPORT)
    {
        hubo.getLegAngles(LEFT, ltLegAngles);
        hubo.getLegAngles(RIGHT, rtLegAngles);
        hubo.huboLegFK(ltFootTF, ltLegAngles, LEFT);
        hubo.huboLegFK(rtFootTF, rtLegAngles, RIGHT);
        world2CenterOfSP.rotate(ltFootTF.rotation());
        world2CenterOfSP.translate((ltFootTF.translation() + rtFootTF.translation()) / 2);
        world2CenterOfSP = world2CenterOfSP * footOffset;
    }
    else if(phase_t == SINGLE_LEFT)
    {
        hubo.getLegAngles(LEFT, ltLegAngles);
        hubo.huboLegFK(ltFootTF, ltLegAngles, LEFT);
        world2CenterOfSP = ltFootTF * footOffset;
    }
    else if(phase_t == SINGLE_RIGHT)
    {
        hubo.getLegAngles(RIGHT, rtLegAngles);
        hubo.huboLegFK(rtFootTF, rtLegAngles, RIGHT);
        world2CenterOfSP = rtFootTF * footOffset;
    }
    else
    {
        fprintf(stderr, "not a support phase\n");
    }

    balPt2COM = (world2CenterOfSP.inverse() * world2COM).translation();
    return balPt2COM;
}

Vector3d getBalancePt2COMTorso(Hubo_Tech &hubo, Eigen::Isometry3d world2COM)
{
    Vector3d balPt2COM;
    Eigen::Isometry3d Neck2BalPt;
    Vector6d ltLegAngles, rtLegAngles;
    Eigen::Isometry3d ltFootTF, rtFootTF, world2CenterOfSP;
    double ankle2Middle = .025;

    ltFootTF = hubo.mSkel->getNode("Body_LAP")->getWorldTransform();
    rtFootTF = hubo.mSkel->getNode("Body_RAP")->getWorldTransform();

    // Create TF from Neck to center of Support Polygon
    world2CenterOfSP = Eigen::Matrix4d::Identity();
    world2CenterOfSP.rotate(ltFootTF.rotation());
    world2CenterOfSP(0,3) = ((ltFootTF(0,3) + ankle2Middle) + (rtFootTF(0,3) + ankle2Middle)) / 2;
    world2CenterOfSP(1,3) = (ltFootTF(1,3) + rtFootTF(1,3)) / 2;
    world2CenterOfSP(2,3) = (ltFootTF(2,3) + rtFootTF(2,3)) / 2;

    balPt2COM = (world2CenterOfSP.inverse() * world2COM).translation();
    return balPt2COM;
}
