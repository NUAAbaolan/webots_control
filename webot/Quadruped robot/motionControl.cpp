#include <motiondefine.h>
#define PI 3.1415926

using namespace std;
using namespace Eigen;
void MotionControl::MotionContr(float tP, float tFGP, Matrix<float, 4, 2> tFSP)
{
    // The parameters for quadruped gait
    initFlag = false;
    timePeriod = tP;
    timeForGaitPeriod = tFGP;
    timeForStancePhase = tFSP;
    timePresent = 0.0;
    timePresentForSwing << 0.0, 0.0, 0.0, 0.0;
    targetCoMVelocity << 0.0, 0.0, 0.0;
    L1 = 0.4;
    L2 = 0.4;
    L3 = 0.0;
    width = 0.48;
    length = 0.6;  
    shoulderPos << width/2, length/2, width/2, -length/2, -width/2, length/2, -width/2, -length/2;  // X-Y: LF, RF, LH, RH

    // The parameters for creeping gait
    // timeOneSwingPeriod = 0.5 * timeForGaitPeriod;
    // L1_creep = 132.0;
    // L2_creep = 112.0;
    // L3_creep = 46.0; 
    // H_onestep = 15.0;
    // k1 = 0.25;
    // k2 = 0.50;
    // k3 = 0.25;
    // initPosS2L << 112.0, -132.0, -46.0, 112.0, 132.0, -46.0, -112.0, -132.0, -46.0, -112.0, 132.0, -46.0;
    // initPosC2L << 198.0, -198.0, 198.0, 198.0, -198.0, -198.0, -198.0, 198.0;
    // initPosC2S << 86.0, -66.0, 86.0, 66.0, -86.0, -66.0, -86.0, 66.0;
    // L_diag = sqrt(initPosC2L(0,0)*initPosC2L(0,0) + initPosC2L(0,1)*initPosC2L(0,1));
    // beta_diag = atan(initPosC2L(0,0)/initPosC2L(0,1));
    // alpha_diag = PI / 2 - beta_diag;
}


void MotionControl::setInitPos(Matrix<float, 4, 3> initPosition)
{
    stancePhaseStartPos = initPosition;
    stancePhaseEndPos = initPosition;
    legPresentPos = initPosition;
    legCmdPos = initPosition;
    targetCoMPosition << 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0;
}

void MotionControl::setCoMVel(Vector<float, 3> tCV)
{
    targetCoMVelocity = tCV;
}

void MotionControl::nextStep()
{
     for(uint8_t legNum=0; legNum<4; legNum++)  // run all 4 legs
    {
        if(timePresent > timeForStancePhase.row(legNum)(0)-timePeriod/2 && timePresent < timeForStancePhase.row(legNum)(1)+timePeriod/2 )
        {     // check timePresent is in stance phase or swing phase, -timePeriod/2 is make sure the equation is suitable
            if(abs(timePresent - timeForStancePhase.row(legNum)(0)) < 1e-4)  // if on the start pos 
            {
                stancePhaseStartPos(legNum) = legCmdPos(legNum);
                for(uint8_t pos=0; pos<3; pos++)
                targetCoMPosition(legNum, pos) = 0.0;
            }
            Matrix<float, 3, 3> trans;
            trans<<cos(targetCoMPosition(legNum,2)), -sin(targetCoMPosition(legNum,2)), targetCoMPosition(legNum,0),
                   sin(targetCoMPosition(legNum,2)), cos(targetCoMPosition(legNum,2)), targetCoMPosition(legNum,1),
                   0, 0, 1;
            Matrix<float, 3, 1> oneShoulderPos_3x1;
            oneShoulderPos_3x1<<shoulderPos.row(legNum)(0), shoulderPos.row(legNum)(1), 1;
            oneShoulderPos_3x1 = trans * oneShoulderPos_3x1;

            if(abs(timePresent - timeForStancePhase.row(legNum)(0)) < 1e-4)  // if on the start pos 
            {
                stancePhaseStartPos(legNum) = legCmdPos(legNum);
                shoulderPos(legNum, 0) = oneShoulderPos_3x1(0);
                shoulderPos(legNum, 1) = oneShoulderPos_3x1(1);
            }
            if(abs(timePresent - timeForStancePhase.row(legNum)(1)) < 1e-4)  // if on the end pos
                stancePhaseEndPos(legNum) = legCmdPos(legNum);

            legCmdPos(legNum, 0) = stancePhaseStartPos(legNum, 0) + (shoulderPos(legNum, 0) - oneShoulderPos_3x1(0));
            legCmdPos(legNum, 1) = stancePhaseStartPos(legNum, 1) + (shoulderPos(legNum, 1) - oneShoulderPos_3x1(1));
            stanceFlag(legNum) = true;
        }
        else
        {
            Matrix<float, 1, 3> swingPhaseVelocity = (stancePhaseEndPos.row(legNum) - stancePhaseStartPos.row(legNum)) / 
                                        (timeForGaitPeriod - (timeForStancePhase(legNum,1) - timeForStancePhase(legNum,0)) - timePeriod);
            
            for(uint8_t pos=0; pos<3; pos++)
            legCmdPos(legNum, pos) = legCmdPos(legNum, pos) - swingPhaseVelocity(pos) * timePeriod;
            
            if( ( timePresentForSwing(legNum) - (timeForGaitPeriod - (timeForStancePhase(legNum,1) - timeForStancePhase(legNum,0)))/2 ) > 1e-4)
            legCmdPos(legNum, 2) -= -0.4;
            if( ( timePresentForSwing(legNum) - (timeForGaitPeriod - (timeForStancePhase(legNum,1) - timeForStancePhase(legNum,0)))/2 ) < -1e-4 && timePresentForSwing(legNum) > 1e-4)
            legCmdPos(legNum, 2) += -0.4;
            stanceFlag(legNum) = false;
        }
    }

    timePresent += timePeriod;
    for(uint8_t leg=0; leg<4; leg++)
    {
        for(uint8_t pos=0; pos<3; pos++)
        {
            targetCoMPosition(leg, pos) += targetCoMVelocity(pos) * timePeriod;
        }
        if(stanceFlag(leg) == 0) timePresentForSwing(leg) += timePeriod;
        else timePresentForSwing(leg) = 0;
    }

    if (abs(timePresent - timeForGaitPeriod - timePeriod) < 1e-4)  // check if present time has reach the gait period                                                               
    {                                                            // if so, set it to 0.0
        timePresent = 0.0;
    }
}

void MotionControl::nextStep1()
{
    if (timePresent < 0.25)
    {    //stance leg is 2 and 3 ,swing leg is 1 and 4
    
        legCmdPos(0, 0) += targetCoMVelocity(0) * timePeriod;
        legCmdPos(0, 1) += targetCoMVelocity(1) * timePeriod;
        legCmdPos(0, 2) += 0.004;
        legCmdPos(3, 0) += targetCoMVelocity(0) * timePeriod;
        legCmdPos(3, 1) += targetCoMVelocity(1) * timePeriod;
        legCmdPos(3, 2) += 0.004;
        
        legCmdPos(1, 0) -= targetCoMVelocity(0) * timePeriod;
        legCmdPos(1, 1) -= targetCoMVelocity(1) * timePeriod;
        legCmdPos(1, 2) += 0.0;
        legCmdPos(2, 0) -= targetCoMVelocity(0) * timePeriod;
        legCmdPos(2, 1) -= targetCoMVelocity(1) * timePeriod;
        legCmdPos(2, 2) += 0.0;
    }
    else
    {
        legCmdPos(1, 0) += targetCoMVelocity(0) * timePeriod;
        legCmdPos(1, 1) += targetCoMVelocity(1) * timePeriod;
        legCmdPos(1, 2) += 0.004;
        legCmdPos(2, 0) += targetCoMVelocity(0) * timePeriod;
        legCmdPos(2, 1) += targetCoMVelocity(1) * timePeriod;
        legCmdPos(2, 2) += 0.004;
        
        legCmdPos(0, 0) -= targetCoMVelocity(0) * timePeriod;
        legCmdPos(0, 1) -= targetCoMVelocity(1) * timePeriod;
        legCmdPos(0, 2) += 0.0;
        legCmdPos(3, 0) -= targetCoMVelocity(0) * timePeriod;
        legCmdPos(3, 1) -= targetCoMVelocity(1) * timePeriod;
        legCmdPos(3, 2) += 0.0;
    }
    timePresent += timePeriod;

}
void MotionControl::inverseKinematics()
{
    // float theta[4][3] = {0};
    float jo_ang[4][3] = {0};
    // float a1[4] = {0};
    // float b1[4] = {0};
    // float c1[4] = {0};
    // float a2[4] = {0};
    // float b2[4] = {0};
    // float c2[4] = {0};
    static int times = 0;
    motorInitPos[0] = 0.0;
    motorInitPos[1] = 0.0;
    motorInitPos[2] = 0.0;
    motorInitPos[3] = 0.0;
    motorInitPos[4] = 0.0;
    motorInitPos[5] = 0.0;
    motorInitPos[6] = 0.0;
    motorInitPos[7] = 0.0;
    motorInitPos[8] = 0.0;
    motorInitPos[9] = 0.0;
    motorInitPos[10] = 0.0;
    motorInitPos[11] = 0.0;

    if(times!=0)
    {
        for(int joints=0; joints<12; joints++)
        {
            jointCmdPosLast[joints] = jointCmdPos[joints];
        }
    }

    for(int leg_num = 0; leg_num < 4; leg_num++)
    {
        float x = legCmdPos(leg_num,0);
        float y = legCmdPos(leg_num,1);
        float z = legCmdPos(leg_num,2);
        float theta0 = atan (- y / z);
        float theta2;
         // = acos ((pow(( sin(theta0) * y - cos(theta0) * z), 2)  + pow(x, 2) - pow(L1, 2) - pow(L2, 2)) / ( 2 * L1 * L2)) - PI ;
        if ( (pow(( sin(theta0) * y - cos(theta0) * z), 2)  + pow(x, 2) - pow(L1, 2) - pow(L2, 2)) / ( 2 * L1 * L2) > 1.0 || (pow(( sin(theta0) * y - cos(theta0) * z), 2)  + pow(x, 2) - pow(L1, 2) - pow(L2, 2)) / ( 2 * L1 * L2) < -1.0)
        {
            theta2 = 0 ;
        }
        else
        {
            theta2 = acos ((pow(( sin(theta0) * y - cos(theta0) * z), 2)  + pow(x, 2) - pow(L1, 2) - pow(L2, 2)) / ( 2 * L1 * L2)) - PI ;
        }
        float cos_theta1 = ((sin (theta2) * y - cos (theta0) * z) * (L1 + L2 * cos (theta2)) + x * L2 * sin(theta2)) / (pow((sin (theta2) * y - cos (theta0) * z), 2) + pow(x, 2));
        float sin_theta1 = ((L1 + L2 * cos (theta2)) * x - (sin (theta2) * y - cos (theta0) * z) * L2 * sin(theta2)) / (pow((sin (theta2) * y - cos (theta0) * z), 2) + pow(x, 2));
        float theta1 = atan (sin_theta1/ cos_theta1);
        jo_ang[leg_num][0] = theta0;
        jo_ang[leg_num][1] = theta1;
        jo_ang[leg_num][2] = theta2;
        // a1[leg_num] = - legCmdPos(leg_num,2);
        // b1[leg_num] = - legCmdPos(leg_num,1);
        // c1[leg_num] = 0;
        // theta[leg_num][0] = asin((c1[leg_num])/sqrt((a1[leg_num])*(a1[leg_num]) + (b1[leg_num])*(b1[leg_num]))) - atan(b1[leg_num]/a1[leg_num]);
        // jo_ang[leg_num][0] = theta[leg_num][0];
        // theta[leg_num][2] = acos(((- legCmdPos(leg_num,2) * cos(theta[leg_num][0]) + legCmdPos(leg_num,1) * sin(theta[leg_num][0]))*
                        // (- legCmdPos(leg_num,2) * cos(theta[leg_num][0]) + legCmdPos(leg_num,1) * sin(theta[leg_num][0])) + 
                        // legCmdPos(leg_num,0)*legCmdPos(leg_num,0) - L1*L1 - L2*L2)/(2*L1*L2));
        // //jo_ang[leg_num][2] = theta[leg_num][2] - PI / 3;
        // jo_ang[leg_num][2] = theta[leg_num][2];
        // a2[leg_num] = -(- legCmdPos(leg_num,2)) * cos(theta[leg_num][0]) - (legCmdPos(leg_num,1)) * sin(theta[leg_num][0]);
        // b2[leg_num] = legCmdPos(leg_num,0);
        // c2[leg_num] = L2 * sin(theta[leg_num][2]);
        
        // theta[leg_num][1] = asin((-c2[leg_num])/sqrt((a2[leg_num])*(a2[leg_num]) + (b2[leg_num])*(b2[leg_num]))) - atan(b2[leg_num]/a2[leg_num]);
        // //jo_ang[leg_num][1] = -(theta[leg_num][1] + PI / 6);
        // jo_ang[leg_num][1] = -theta[leg_num][1];
        // if (jo_ang[leg_num][1] > 2.0)
        // {
          // jo_ang[leg_num][1] = jo_ang[leg_num][1] - PI;
        // }
    }

    joint_cmd_pos[0] = jo_ang[0][0];
    joint_cmd_pos[1] = jo_ang[0][1];
    joint_cmd_pos[2] = jo_ang[0][2];
    joint_cmd_pos[3] = jo_ang[1][0];
    joint_cmd_pos[4] = jo_ang[1][1];
    joint_cmd_pos[5] = jo_ang[1][2];
    joint_cmd_pos[6] = jo_ang[2][0];
    joint_cmd_pos[7] = jo_ang[2][1];
    joint_cmd_pos[8] = jo_ang[2][2];
    joint_cmd_pos[9] = jo_ang[3][0];
    joint_cmd_pos[10] = jo_ang[3][1];
    joint_cmd_pos[11] = jo_ang[3][2];
    

    jointCmdPos[0] = motorInitPos[0] + jo_ang[0][0];
    jointCmdPos[1] = motorInitPos[1] + jo_ang[0][1];
    jointCmdPos[2] = motorInitPos[2] + jo_ang[0][2];
    jointCmdPos[3] = motorInitPos[3] + jo_ang[1][0];
    jointCmdPos[4] = motorInitPos[4] + jo_ang[1][1];
    jointCmdPos[5] = motorInitPos[5] + jo_ang[1][2];
    jointCmdPos[6] = motorInitPos[6] + jo_ang[2][0];
    jointCmdPos[7] = motorInitPos[7] + jo_ang[2][1];
    jointCmdPos[8] = motorInitPos[8] + jo_ang[2][2];
    jointCmdPos[9] = motorInitPos[9] + jo_ang[3][0];
    jointCmdPos[10] = motorInitPos[10] + jo_ang[3][1];
    jointCmdPos[11] = motorInitPos[11] + jo_ang[3][2];

    if(times!=0)
    {
        for(int joints=0; joints<12; joints++)
        {
            jointCmdVel[joints] = (jointCmdPos[joints] - jointCmdPosLast[joints]) / timePeriod;
        }
    }
    else
    {
        for(int joints=0; joints<12; joints++)
        {
            jointCmdVel[joints] = 0;
        }
    }
    times++;
}

void MotionControl::forwardKinematics()
{
    float joint_pres_pos[4][3];
    joint_pres_pos[0][0] = jointPresentPos[0];
    joint_pres_pos[0][1] = jointPresentPos[1];
    joint_pres_pos[0][2] = jointPresentPos[2];
    joint_pres_pos[1][0] = jointPresentPos[3];
    joint_pres_pos[1][1] = jointPresentPos[4];
    joint_pres_pos[1][2] = jointPresentPos[5];
    joint_pres_pos[2][0] = jointPresentPos[6];
    joint_pres_pos[2][1] = jointPresentPos[7];
    joint_pres_pos[2][2] = jointPresentPos[8];
    joint_pres_pos[3][0] = jointPresentPos[9];
    joint_pres_pos[3][1] = jointPresentPos[10];
    joint_pres_pos[3][2] = jointPresentPos[11];

    joint_pre_pos[0] = joint_pres_pos[0][0];
    joint_pre_pos[1] = joint_pres_pos[0][1];
    joint_pre_pos[2] = joint_pres_pos[0][2];
    joint_pre_pos[3] = joint_pres_pos[1][0];
    joint_pre_pos[4] = joint_pres_pos[1][1];
    joint_pre_pos[5] = joint_pres_pos[1][2];
    joint_pre_pos[6] = joint_pres_pos[2][0];
    joint_pre_pos[7] = joint_pres_pos[2][1];
    joint_pre_pos[8] = joint_pres_pos[2][2];
    joint_pre_pos[9] = joint_pres_pos[3][0];
    joint_pre_pos[10] = joint_pres_pos[3][1];
    joint_pre_pos[11] = joint_pres_pos[3][2];

    for(int leg_nums = 0; leg_nums < 4; leg_nums++)
    {
        legPresentPos(leg_nums,0) = -L1 * sin(joint_pres_pos[leg_nums][1]) - L2 * sin(joint_pres_pos[leg_nums][1] + joint_pres_pos[leg_nums][2]);
        legPresentPos(leg_nums,1) = L1 * sin(joint_pres_pos[leg_nums][0]) * cos(joint_pres_pos[leg_nums][1]) + L2 * sin(joint_pres_pos[leg_nums][0]) * cos(joint_pres_pos[leg_nums][1] + joint_pres_pos[leg_nums][2]);
        legPresentPos(leg_nums,2) = -L1 * cos(joint_pres_pos[leg_nums][0]) * cos(joint_pres_pos[leg_nums][1]) - L2 * cos(joint_pres_pos[leg_nums][0]) * cos(joint_pres_pos[leg_nums][1] + joint_pres_pos[leg_nums][2]);
        
        // legPresentPos(leg_nums,0) = L2 * cos(joint_pres_pos[leg_nums][0]) * cos(joint_pres_pos[leg_nums][1] + joint_pres_pos[leg_nums][2]) + L1 * cos(joint_pres_pos[leg_nums][0]) * cos(joint_pres_pos[leg_nums][1]);
        // legPresentPos(leg_nums,1) = L2 * sin(joint_pres_pos[leg_nums][0]) * cos(joint_pres_pos[leg_nums][1] + joint_pres_pos[leg_nums][2]) + L1 * sin(joint_pres_pos[leg_nums][0]) * cos(joint_pres_pos[leg_nums][1]);
        // legPresentPos(leg_nums,2) = L2 * sin(joint_pres_pos[leg_nums][1] + joint_pres_pos[leg_nums][2]) + L1 * sin(joint_pres_pos[leg_nums][1]);
        leg2CoMPrePos(leg_nums,0) = shoulderPos(leg_nums,1) + legPresentPos(leg_nums,2);
        leg2CoMPrePos(leg_nums,1) = shoulderPos(leg_nums,0) + legPresentPos(leg_nums,1);
        leg2CoMPrePos(leg_nums,2) = legPresentPos(leg_nums,0);
        
        legCmdPos(leg_nums,0) = legPresentPos(leg_nums,0);
        legCmdPos(leg_nums,1) = legPresentPos(leg_nums,1);
        legCmdPos(leg_nums,2) = legPresentPos(leg_nums,2);   
    }
}

void MotionControl::jacobians()
{
    jacobian(0 ,0) = -L2 * sin(joint_pre_pos[0]) * cos(joint_pre_pos[1] + joint_pre_pos[2]) - L1 * sin(joint_pre_pos[0]) * cos(joint_pre_pos[1]);
    jacobian(0 ,1) =  L2 * cos(joint_pre_pos[0]) * cos(joint_pre_pos[1] + joint_pre_pos[2]) + L1 * cos(joint_pre_pos[0]) * cos(joint_pre_pos[1]);
    jacobian(0 ,2) = 0;
    jacobian(0 ,3) = -L2 * cos(joint_pre_pos[0]) * sin(joint_pre_pos[1] + joint_pre_pos[2]) - L1 * cos(joint_pre_pos[0]) * sin(joint_pre_pos[1]);
    jacobian(0 ,4) = -L2 * sin(joint_pre_pos[0]) * sin(joint_pre_pos[1] + joint_pre_pos[2]) - L1 * sin(joint_pre_pos[0]) * sin(joint_pre_pos[1]);
    jacobian(0 ,5) = L2 * cos(joint_pre_pos[1] + joint_pre_pos[2]) + L1 * cos(joint_pre_pos[1]);
    jacobian(0 ,6) = -L2 * cos(joint_pre_pos[0]) * sin(joint_pre_pos[1] + joint_pre_pos[2]);
    jacobian(0 ,7) = -L2 * sin(joint_pre_pos[0]) * sin(joint_pre_pos[1] + joint_pre_pos[2]);
    jacobian(0 ,8) = L2 * cos(joint_pre_pos[1] + joint_pre_pos[2]);

    jacobian(1 ,0)= -L2 * sin(joint_pre_pos[3]) * cos(joint_pre_pos[4] + joint_pre_pos[5]) - L1 * sin(joint_pre_pos[3]) * cos(joint_pre_pos[4]);
    jacobian(1 ,1) =  L2 * cos(joint_pre_pos[3]) * cos(joint_pre_pos[4] + joint_pre_pos[5]) + L1 * cos(joint_pre_pos[3]) * cos(joint_pre_pos[4]);
    jacobian(1 ,2) = 0;
    jacobian(1 ,3) = -L2 * cos(joint_pre_pos[3]) * sin(joint_pre_pos[4] + joint_pre_pos[5]) - L1 * cos(joint_pre_pos[3]) * sin(joint_pre_pos[4]);
    jacobian(1 ,4) = -L2 * sin(joint_pre_pos[3]) * sin(joint_pre_pos[4] + joint_pre_pos[5]) - L1 * sin(joint_pre_pos[3]) * sin(joint_pre_pos[4]);
    jacobian(1 ,5) = L2 * cos(joint_pre_pos[4] + joint_pre_pos[5]) + L1 * cos(joint_pre_pos[4]);
    jacobian(1 ,6) = -L2 * cos(joint_pre_pos[3]) * sin(joint_pre_pos[4] + joint_pre_pos[5]);
    jacobian(1 ,7) = -L2 * sin(joint_pre_pos[3]) * sin(joint_pre_pos[4] + joint_pre_pos[5]);
    jacobian(1 ,8) = L2 * cos(joint_pre_pos[4] + joint_pre_pos[5]);

    jacobian(2 ,0)= -L2 * sin(joint_pre_pos[6]) * cos(joint_pre_pos[7] + joint_pre_pos[8]) - L1 * sin(joint_pre_pos[6]) * cos(joint_pre_pos[7]);
    jacobian(2 ,1) =  L2 * cos(joint_pre_pos[6]) * cos(joint_pre_pos[7] + joint_pre_pos[8]) + L1 * cos(joint_pre_pos[6]) * cos(joint_pre_pos[7]);
    jacobian(2 ,2) = 0;
    jacobian(2 ,3) = -L2 * cos(joint_pre_pos[6]) * sin(joint_pre_pos[7] + joint_pre_pos[8]) - L1 * cos(joint_pre_pos[6]) * sin(joint_pre_pos[7]);
    jacobian(2 ,4) = -L2 * sin(joint_pre_pos[6]) * sin(joint_pre_pos[7] + joint_pre_pos[8]) - L1 * sin(joint_pre_pos[6]) * sin(joint_pre_pos[7]);
    jacobian(2 ,5) = L2 * cos(joint_pre_pos[7] + joint_pre_pos[8]) + L1 * cos(joint_pre_pos[7]);
    jacobian(2 ,6) = -L2 * cos(joint_pre_pos[6]) * sin(joint_pre_pos[7] + joint_pre_pos[8]);
    jacobian(2 ,7) = -L2 * sin(joint_pre_pos[6]) * sin(joint_pre_pos[7] + joint_pre_pos[8]);
    jacobian(2 ,8) = L2 * cos(joint_pre_pos[7] + joint_pre_pos[8]);

    jacobian(3, 0) = -L2 * sin(joint_pre_pos[9]) * cos(joint_pre_pos[10] + joint_pre_pos[11]) - L1 * sin(joint_pre_pos[9]) * cos(joint_pre_pos[10]);
    jacobian(3, 1) =  L2 * cos(joint_pre_pos[9]) * cos(joint_pre_pos[10] + joint_pre_pos[11]) + L1 * cos(joint_pre_pos[9]) * cos(joint_pre_pos[10]);
    jacobian(3, 2) = 0;
    jacobian(3, 3) = -L2 * cos(joint_pre_pos[9]) * sin(joint_pre_pos[10] + joint_pre_pos[11]) - L1 * cos(joint_pre_pos[9]) * sin(joint_pre_pos[10]);
    jacobian(3, 4) = -L2 * sin(joint_pre_pos[9]) * sin(joint_pre_pos[10] + joint_pre_pos[11]) - L1 * sin(joint_pre_pos[9]) * sin(joint_pre_pos[10]);
    jacobian(3, 5) = L2 * cos(joint_pre_pos[10] + joint_pre_pos[11]) + L1 * cos(joint_pre_pos[10]);
    jacobian(3, 6) = -L2 * cos(joint_pre_pos[9]) * sin(joint_pre_pos[10] + joint_pre_pos[11]);
    jacobian(3, 7) = -L2 * sin(joint_pre_pos[9]) * sin(joint_pre_pos[10] + joint_pre_pos[11]);
    jacobian(3, 8) = L2 * cos(joint_pre_pos[10] + joint_pre_pos[11]);
}

void MotionControl::updateState()
{
    
}

void MotionControl::vmc()
{
    float kx = 500.0;
    float ky = 500.0;
    // float kw = 0.0;
    if (stanceFlag[0] == 0)
    {
        float xf= leg2CoMPrePos(0, 0);
        float yf= leg2CoMPrePos(0, 1);
        float zf= leg2CoMPrePos(0, 2);
        float xh= leg2CoMPrePos(3, 0);
        float yh= leg2CoMPrePos(3, 1);
        float zh= leg2CoMPrePos(3, 2);
        A << 1, 0, 0, 1, 0, 0,
             0, 1, 0, 0, 1, 0, 
             0, 0, 1, 0, 0, 1,
             0, -zf, yf, 0, -zh, yh, 
             zf, 0, -xf, zh, 0, -xh, 
             -yh, -xf, 0, yh, xh, 0;
        Matrix<float, 3, 3>temp_Matrix;
        temp_Matrix << jacobian(3 ,0), jacobian(3 ,1), jacobian(3 ,2),
                       jacobian(3 ,3), jacobian(3 ,4), jacobian(3 ,5),
                       jacobian(3 ,6), jacobian(3 ,7), jacobian(3 ,8);
        Vector<float, 3>temp_vel;
        temp_vel(0) = (jointPresentVel(9) - jointPresentVel(10))/2;
        temp_vel(1) = -(jointPresentVel(9) + jointPresentVel(10))/2;
        temp_vel(2) = -jointPresentVel(11);
        Vector<float, 3>temp_comvel;
        temp_comvel = -temp_Matrix*temp_vel * 0.000001;
        presentCoMVelocity[0] = temp_comvel(0);
        presentCoMVelocity[1] = temp_comvel(1);
        // cout << "vel:"<<temp_comvel.transpose()<<endl;
        float Fx = kx * (targetCoMVelocity[0] - presentCoMVelocity[0]);
        float Fy = ky * (targetCoMVelocity[1] - presentCoMVelocity[1]);
        // float tao_z = kw * (presentCoMVelocity[2] - targetCoMVelocity[2]);
        // float Fx = 0.0;
        // float Fy = 0.0;
        float tao_z = 0.0;
        B << Fx, Fy, 9.8 * 2.5, 0, 0, tao_z;
        // cout << "force: " <<  B.transpose() << endl;
        double u=0.7;
        double k=2;
        a << 1, 0, 0, 1, 0, 0, 
            0, 1, 0, 0, 1, 0, 
            -1, -1, -sqrt(2)*u/k, 0, 0, 0, 
            0, 0, 0, 1, 1, -sqrt(2)*u/k;
        b << Fx, Fy, 0, 0;
    }
    else
    {
        float xf= leg2CoMPrePos(1, 0);
        float yf= leg2CoMPrePos(1, 1);
        float zf= leg2CoMPrePos(1, 2);
        float xh= leg2CoMPrePos(2, 0);
        float yh= leg2CoMPrePos(2, 1);
        float zh= leg2CoMPrePos(2, 2);
        A << 1, 0, 0, 1, 0, 0,
             0, 1, 0, 0, 1, 0, 
             0, 0, 1, 0, 0, 1, 
             0, -zf, yf, 0, -zh, yh, 
             zf, 0, -xf, zh, 0, -xh, 
             -yh, -xf, 0, yh, xh, 0;
        Matrix<float, 3, 3>temp_Matrix;
        temp_Matrix << jacobian(2 ,0), jacobian(2 ,1), jacobian(2 ,2),
                       jacobian(2 ,3), jacobian(2 ,4), jacobian(2 ,5),
                       jacobian(2 ,6), jacobian(2 ,7), jacobian(2 ,8);
        Vector<float, 3>temp_vel;
        temp_vel(0) = (jointPresentVel(6) - jointPresentVel(7))/2;
        temp_vel(1) = -(jointPresentVel(6) + jointPresentVel(7))/2;
        temp_vel(2) = jointPresentVel(8);
        Vector<float, 3>temp_comvel;
        temp_comvel = -temp_Matrix * temp_vel *  0.000001;
        presentCoMVelocity[0] = temp_comvel(0);
        presentCoMVelocity[1] = temp_comvel(1);
        // cout << "vel:"<<temp_comvel.transpose()<<endl;
        float Fx = kx * (targetCoMVelocity[0] - presentCoMVelocity[0]);
        float Fy = ky * (targetCoMVelocity[1] - presentCoMVelocity[1]);
        // float tao_z = kw * (presentCoMVelocity[2] - targetCoMVelocity[2]);
        // float Fx = 0.0;
        // float Fy = 0.0;
        float tao_z = -0.0;
        B << Fx, Fy, 9.8 * 2.5, 0, 0, tao_z;
        // cout << "force: " << B.transpose() << endl;
        double u=0.7;
        double k=2;
        a << 1, 0, 0, 1, 0, 0, 
            0, 1, 0, 0, 1, 0, 
            1, -1, -sqrt(2)*u/k, 0, 0, 0, 
            0, 0, 0, -1, 1, -sqrt(2)*u/k;
        b << Fx, Fy, 0, 0;
    }
    int m = 4;
    MatrixXf n(6+m, 6+m);
    n.block(0, 0, 6, 6) = A.transpose()*A;
    n.block(6, 0, m, 6) = a;
    n.block(0, 6, 6, m) = -a.transpose(); 
    n.block(6, 6, m, m) = MatrixXf::Zero(m,m);
    VectorXf nt(6+m, 1);
    nt.head(6) = A.transpose()*B;
    nt.tail(m) = b; 
    VectorXf x(6+m, 1);
    x = n.colPivHouseholderQr().solve(nt);
    Vector<float, 6> jacobian_Force;
    jacobian_Force = x.head(6);
    Vector<float, 6> temp_Force;
    temp_Force = x.head(6);
    // cout << "force :" << (A * temp_Force).transpose() << endl;
    Matrix<float, 6, 6>jacobian_Matrix;
    Vector<float, 12>jacobian_torque;
    if (stanceFlag[0] == 0)
    {
        jacobian_Matrix.block(0,0,3,3) << jacobian(0 ,0), jacobian(0 ,1), jacobian(0 ,2),
                                          jacobian(0 ,3), jacobian(0 ,4), jacobian(0 ,5),
                                          jacobian(0 ,6), jacobian(0 ,7), jacobian(0 ,8);
        jacobian_Matrix.block(3,3,3,3) << jacobian(3 ,0), jacobian(3 ,1), jacobian(3 ,2),
                                          jacobian(3 ,3), jacobian(3 ,4), jacobian(3 ,5),
                                          jacobian(3 ,6), jacobian(3 ,7), jacobian(3 ,8);
        jacobian_Matrix.block(0,3,3,3) = MatrixXf::Zero(3, 3);
        jacobian_Matrix.block(3,0,3,3) = MatrixXf::Zero(3, 3);
        Vector<float, 6> temp_torque;
        temp_torque = jacobian_Matrix * temp_Force;
        // cout<<"stance left: "<<temp_Force.transpose() << endl;
        jacobian_torque.head(3) = temp_torque.head(3);
        jacobian_torque.tail(3) = temp_torque.tail(3);
        jacobian_torque.segment(3, 6) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        
    }
    else
    {
        jacobian_Matrix.block(0,0,3,3) << jacobian(1 ,0), jacobian(1 ,1), jacobian(1 ,2),
                                          jacobian(1 ,3), jacobian(1 ,4), jacobian(1 ,5),
                                          jacobian(1 ,6), jacobian(1 ,7), jacobian(1 ,8);
        jacobian_Matrix.block(3,3,3,3) << jacobian(2 ,0), jacobian(2 ,1), jacobian(2 ,2),
                                          jacobian(2 ,3), jacobian(2 ,4), jacobian(2 ,5),
                                          jacobian(2 ,6), jacobian(2 ,7), jacobian(2 ,8);
        jacobian_Matrix.block(0,3,3,3) = MatrixXf::Zero(3, 3);
        jacobian_Matrix.block(3,0,3,3) = MatrixXf::Zero(3, 3);
        Vector<float, 6> temp_torque;
        temp_torque = jacobian_Matrix * temp_Force;
        // cout<<"stance right: "<<temp_Force.transpose() << endl;

        jacobian_torque.head(3) << 0.0, 0.0, 0.0;
        jacobian_torque.tail(3) << 0.0, 0.0, 0.0;
        jacobian_torque.segment(3, 6) = temp_torque;
    }   
    jacobian_motortorque[0] = 0.5 * jacobian_torque(0) + 0.5 * jacobian_torque(1);
    jacobian_motortorque[1] = -0.5 * jacobian_torque(0) + 0.5 * jacobian_torque(1);
    jacobian_motortorque[2] = -jacobian_torque(2);
    jacobian_motortorque[3] = 0.5 * jacobian_torque(3) - 0.5 * jacobian_torque(4);
    jacobian_motortorque[4] = -0.5 * jacobian_torque(3) - 0.5 * jacobian_torque(4);
    jacobian_motortorque[5] = jacobian_torque(5);
    jacobian_motortorque[6] = 0.5 * jacobian_torque(6) - 0.5 * jacobian_torque(7);
    jacobian_motortorque[7] = -0.5 * jacobian_torque(6) - 0.5 * jacobian_torque(7);
    jacobian_motortorque[8] = jacobian_torque(8);
    jacobian_motortorque[9] = 0.5 * jacobian_torque(9) - 0.5 * jacobian_torque(10);
    jacobian_motortorque[10] = -0.5 * jacobian_torque(9) - 0.5 * jacobian_torque(10);
    jacobian_motortorque[11] = -jacobian_torque(11);
    for (int i = 0 ;i <12 ; i++)
    {
        jacobian_motortorque[i] = 0.000001 * jacobian_motortorque[i];
    }
}

void MotionControl::setInitial()
{
    // float jo_pos[12] = {0.0};
    // float ap,bt,gm;
    // float T_cal = 0.00;

    // for(int item = 0; item < 200; item++)
    // {
    //     ap = 0.0;
    //     bt = (PI/6) / T_cycle * T_cal;
    //     gm = (PI/3) / T_cycle * T_cal;
    //     T_cal = T_cal + timePeriod;
    // }
    
    // jointCmdPos[0] =  -1.3315 - ap + bt;
    // jointCmdPos[1] =  1.4788 + ap + bt;
    // jointCmdPos[2] = 0.7854 - gm;
    // jointCmdPos[3] = 0.5292 - ap - bt;
    // jointCmdPos[4] = 0.7670 + ap - bt;
    // jointCmdPos[5] = -0.7854 + gm;
    // jointCmdPos[6] = 1.7058 + ap - bt;
    // jointCmdPos[7] = -0.3421 - ap - bt;
    // jointCmdPos[8] = -0.7854 + gm;
    // jointCmdPos[9] = 0.2770 + ap - bt;
    // jointCmdPos[10] = -0.2954 - ap - bt;
    // jointCmdPos[11] = 0.7854 - gm;
}

void MotionControl::pid()
{
    Vector<float, 12> temp_jointCmdPos, temp_jointCmdVel;
    for(uint8_t joints=0; joints<12; joints++)
    {
        temp_jointCmdPos(joints) = jointCmdPos[joints];
        temp_jointCmdVel(joints) = jointCmdVel[joints];
    }
    // cout<<"jointPresentPos: "<<jointPresentPos.transpose()<<endl;
    // cout<<"yawVelocity: "<<mc.yawVelocity<<endl;
    //cout<<"jointPresentVel: "<<mc.jointPresentVel.transpose()<<endl;
    // cout<<"jointCmdPos: "<<temp_jointCmdPos.transpose()<<endl;
    //cout<<"jointCmdVel: "<<temp_jointCmdVel.transpose()<<endl;
    Vector<float, 12> temp_motorCmdTorque;
    temp_motorCmdTorque = 50000000 * (temp_jointCmdPos - jointPresentPos) + 0.2* (temp_jointCmdVel - jointPresentVel);
    // cout << "torque:  " << temp_motorCmdTorque.transpose() << endl;
    // float motorCmdTorque[12];
    for(uint8_t joints=0; joints<12; joints++)
    {
        pid_motortorque[joints] = temp_motorCmdTorque(joints);
    }
}
