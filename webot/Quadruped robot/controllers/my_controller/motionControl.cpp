#include <motiondefine.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Sparse>
#include <Eigen/SVD>
#include <math.h>
#define PI 3.1415926
#define TIME_STEP 10

using namespace std;
using namespace Eigen;

void MotionControl::MotionContr(float tP, float tFGP, Matrix<float, 4, 2> tFSP, Robot *robot)
{
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
    width = 0.6;
    length = 0.48;  
    shoulderPos << width/2, length/2, width/2, -length/2, -width/2, length/2, -width/2, -length/2;  // X-Y: LF, RF, LH, RH
    for (int i=0; i<12; i++)
    {
      Ps[i] = robot->getPositionSensor(positionName[i]);
      Ps[i]->enable(TIME_STEP); 
      Tor[i] = robot->getMotor(motorName[i]);
    }
    for (int i=0; i<4; i++)
    {
      Ts[i] = robot->getTouchSensor(touchsensorName[i]);
      Ts[i]->enable(TIME_STEP);   
    }
   imu = robot->getInertialUnit("inertial unit");
   imu->enable(TIME_STEP);
}
void MotionControl::Sensor_update()
{
    Vector<float, 12> jointLastPos;
    for (int i = 0; i < 12; i++)
    {
        jointLastPos(i) = jointPresentPos(i);
    }
    Vector<float, 3> Lastimu_num;
    for (int i = 0; i < 3; i++)
    {
        Lastimu_num(i) = imu_num(i);
    }
    
    for (int i=0; i<12; i++)
    {
        jointPresentPos(i) = Ps[i]->getValue();
    }
    float temp_touch1 = Ts[0]->getValue();
    float temp_touch2 = Ts[1]->getValue();
    float temp_touch3 = Ts[2]->getValue();
    float temp_touch4 = Ts[3]->getValue();
    state << temp_touch1, temp_touch2, temp_touch3, temp_touch4;
    for (int i = 0; i < 3; i++)
    {
        imu_num(i) = imu->getRollPitchYaw()[i];
    } 
    cout << "imu:   "<< imu_num.transpose()<<endl;       
    for (int i = 0; i < 12; i++)
    {
        jointPresentVel(i) = (jointPresentPos(i) - jointLastPos(i))/ timePeriod;
    }      

    for (int i = 0; i < 3; i++)
    {
        imuVel(i) = (imu_num(i) - Lastimu_num(i)) / timePeriod;
    } 
}
void MotionControl::state_judgement()
{
    Vector<float, 4> temp_4leg;
    temp_4leg << 1, 1, 1, 1;
    Vector<float, 4> temp_3leg1;
    temp_3leg1 << 0, 1, 1, 1;
    Vector<float, 4> temp_3leg2;
    temp_3leg2 << 1, 0, 1, 1;
    Vector<float, 4> temp_3leg3;
    temp_3leg3 << 1, 1, 0, 1;
    Vector<float, 4> temp_3leg4;
    temp_3leg4 << 1, 1, 1, 0;
    Vector<float, 4> temp_2leg1;
    temp_2leg1 << 1, 0, 0, 1;
    Vector<float, 4> temp_2leg2;
    temp_2leg2 << 0, 1, 1, 0;
    if (state == temp_4leg)
      state_val = 1;
    else if(state == temp_3leg1)
      state_val = 2;
    else if(state == temp_3leg2)
      state_val = 3;
    else if(state == temp_3leg3)
      state_val = 4;
    else if(state == temp_3leg4)
      state_val = 5;
    else if(state == temp_2leg1)
      state_val = 6;
    else if(state == temp_2leg2)
      state_val = 7;    
}
void MotionControl::Setjoint()
{
    for (int i= 0 ; i<12; i++)
    {
        if (jacobian_torque(i) < -2000 || jacobian_torque(i) > 2000)
        jacobian_torque(i) = 0.0;
    }
    Vector<float, 12> temp_jointCmdPos, temp_jointCmdVel;
    for(uint8_t joints=0; joints<12; joints++)
    {
        temp_jointCmdPos(joints) = jointCmdPos[joints];
        temp_jointCmdVel(joints) = jointCmdVel[joints];
    }
    Vector<float, 12> temp_motorCmdTorque;
    temp_motorCmdTorque = 0.0* (temp_jointCmdPos - jointPresentPos) + 0.0* (temp_jointCmdVel - jointPresentVel);
    jacobian_torque += temp_motorCmdTorque;
    for (int i = 0; i < 12; i++)
    {
        Tor[i]->setTorque(jacobian_torque(i));
    }
    // if (swingFlag == 0)
    // {
        // for (int i= 0 ; i<12; i++)
        // {
            // if (i < 3 || i > 8)
            // Tor[i]->setTorque(jacobian_torque(i));
            // else 
            // Tor[i]->setPosition(jointCmdPos[i]);
        // }
    // }
    // else
    // {
        // for (int i= 0 ; i<12; i++)
        // {
            // if (i >2 && i <9)
            // Tor[i]->setTorque(jacobian_torque(i));
            // else 
            // Tor[i]->setPosition(jointCmdPos[i]);
        // }
    // }
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

void MotionControl::State_start()
{
    Sensor_update(); 
    for (int i = 0; i < 12 ;i++)
    {
        jointCmdPos[i] = jointPresentPos(i);
    }
    for (int i= 0; i<12; i++)
    {
        Tor[i]->setPosition(jointCmdPos[i]);
    }
    Start_time += timePeriod;
}
void MotionControl::stance_VMC()
{
    static float target_taoz = 0.0;
    float G = 66;
    float kx = 1500;
    float ky = 200 ;
    float kz = 800;
    float dx = 100;
    float dy = 550;
    float dz = 120;
    float k_taox = 1460;
    float k_taoy = 1400;
    float k_taoz = 1200;
    float d_taox = 130;//110
    float d_taoy = 81;
    float d_taoz = 100;
    target_taoz = target_taoz + targetCoMVelocity[2] * timePeriod;
    //1:y,  2:x,  3:z
    float tao_x = k_taox * (0.0 - imu_num[1]) + d_taox * (0 -imuVel(1));
    float tao_y = k_taoy * (0.0 - imu_num[0]) + d_taoy * (0 -imuVel(0));
    float tao_z = k_taoz * (target_taoz - imu_num[2]) + d_taoz * (targetCoMVelocity[2]  -imuVel(2)); 
    if (swingFlag == 0)
    {
        float xf = leg2CoMPrePos(0, 0);
        float yf = leg2CoMPrePos(0, 1);
        float zf = leg2CoMPrePos(0, 2);
        float xh = leg2CoMPrePos(3, 0);
        float yh = leg2CoMPrePos(3, 1);
        float zh = leg2CoMPrePos(3, 2);
        A << 1, 0, 0, 1, 0, 0,
             0, 1, 0, 0, 1, 0,
             0, 0, 1, 0, 0, 1,
             0, -zf, yf, 0, -zh, yh, 
             zf, 0, -xf, zh, 0, -xh, 
             -yf, xf, 0, -yh, xh, 0;
        Matrix<float, 3, 3>temp_Matrix;
        temp_Matrix << jacobian(3 ,0), jacobian(3 ,1), jacobian(3 ,2),
                       jacobian(3 ,3), jacobian(3 ,4), jacobian(3 ,5),
                       jacobian(3 ,6), jacobian(3 ,7), jacobian(3 ,8);
        Vector<float, 3>temp_vel;
        temp_vel(0) = jointPresentVel(9);
        temp_vel(1) = jointPresentVel(10);
        temp_vel(2) = jointPresentVel(11);
        Vector<float, 3>temp_comvel;
        temp_comvel = -temp_Matrix*temp_vel;
        presentCoMVelocity[0] = temp_comvel(0);
        presentCoMVelocity[1] = temp_comvel(1);
        presentCoMVelocity[2] = temp_comvel(2);
        float Fx = -kx * (-0.055- legPresentPos(3,0)) + dx * (targetCoMVelocity[0] - presentCoMVelocity[0]);
        float Fz = -kz * (-0.640955 - legPresentPos(3,2)) + dz * (0 - presentCoMVelocity(2));
        float Fy = -ky * (-5.95758e-06  -legPresentPos(3, 1)) + dy * (targetCoMVelocity[1] - presentCoMVelocity[1]);
        B << Fx - 9.81 * G * sin(-imu_num(1)), Fy, 9.81 * G * cos(-imu_num(1)) + Fz, tao_x, tao_y, tao_z;
    }
    else
    {
        float xf = leg2CoMPrePos(1, 0);
        float yf = leg2CoMPrePos(1, 1);
        float zf = leg2CoMPrePos(1, 2);
        float xh = leg2CoMPrePos(2, 0);
        float yh = leg2CoMPrePos(2, 1);
        float zh = leg2CoMPrePos(2, 2);
        A << 1, 0, 0, 1, 0, 0,
             0, 1, 0, 0, 1, 0,
             0, 0, 1, 0, 0, 1,
             0, -zf, yf, 0, -zh, yh, 
             zf, 0, -xf, zh, 0, -xh, 
             -yf, xf, 0, -yh, xh, 0;
        Matrix<float, 3, 3>temp_Matrix;
        temp_Matrix << jacobian(2 ,0), jacobian(2 ,1), jacobian(2 ,2),
                       jacobian(2 ,3), jacobian(2 ,4), jacobian(2 ,5),
                       jacobian(2 ,6), jacobian(2 ,7), jacobian(2 ,8);
        Vector<float, 3>temp_vel;
        temp_vel(0) = jointPresentVel(6);
        temp_vel(1) = jointPresentVel(7);
        temp_vel(2) = jointPresentVel(8);
        Vector<float, 3>temp_comvel;
        temp_comvel = -temp_Matrix * temp_vel;
        presentCoMVelocity[0] = temp_comvel(0);
        presentCoMVelocity[1] = temp_comvel(1);
        presentCoMVelocity[2] = temp_comvel(2);        
        float Fx = -kx * (-0.055 - legPresentPos(2,0)) + dx * (targetCoMVelocity[0] - presentCoMVelocity[0]);
        float Fz = -kz * (-0.640955 - legPresentPos(2,2)) + dz * (0 - presentCoMVelocity(2));
        float Fy = -ky * (-5.95758e-06  -legPresentPos(2, 1)) + dy * (targetCoMVelocity[1] - presentCoMVelocity[1]);
        B << Fx - 9.81 * G * sin(-imu_num(1)), Fy, 9.81 * G * cos(-imu_num(1)) + Fz, tao_x, tao_y, tao_z;
    }
    Matrix<float, 12, 6> temp_matrix;
    float temp = 0.08;
    temp_matrix.block(0,0,6,6) = A;
    temp_matrix.block(6,0,6,6) = temp * MatrixXf::Identity(6, 6);
    temp_matrix(8, 2) = 0.1 * temp_matrix(8, 2);
    temp_matrix(11, 5) = 0.1 * temp_matrix(11, 5);
    temp_matrix(6, 0) = 10 * temp_matrix(6, 0);
    temp_matrix(9, 3) = 10 * temp_matrix(9, 3);
    Vector<float, 12>temp_vector;
    temp_vector.head(6) = B;
    temp_vector.tail(6) << 0,0,0,0,0,0;                               
    Vector<float, 6> temp_Force;
    temp_Force =  temp_matrix.jacobiSvd(ComputeThinU | ComputeThinV).solve(temp_vector);
    Matrix<float, 6, 6>jacobian_Matrix;
    if (swingFlag == 0)
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
        temp_torque = -jacobian_Matrix.transpose() * temp_Force;
        jacobian_torque.head(3) = temp_torque.head(3);
        jacobian_torque.tail(3) = temp_torque.tail(3);
        
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
        temp_torque = -jacobian_Matrix.transpose() * temp_Force;
        jacobian_torque.segment(3, 6) = temp_torque;        
    }  
}
void MotionControl::swing_VMC()
{
    float H = 0.2;
    float T = 0.32;
    float S = T * targetCoMVelocity[0]/2; 
    float t = fly_time ; 
    float x = S * (t/T-sin(2*PI*t/T)/(2*PI));
    float vx = S * (1/T-cos(2*PI*t/T)/T);
    float sgn;
    if ( t >= 0 && t < T/2)
    {
       sgn = 1;
    }
    else
    {
       sgn = -1;
    }
    float z = H * (sgn * (2*(t/T-sin(4*PI*t/T)/(4*PI))-1)+1);
    float vz = H * (sgn *(2*(1/T-cos(4*PI*t/T)/T)-1)+1);
    float y = targetCoMVelocity[1] * t;
    float vy = targetCoMVelocity[1];
    float swingfx_kp = 120;
    float swingfy_kp = 110;
    float swingfz_kp = 160; 
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   ;
    float swingfx_kd = 120;
    float swingfy_kd = 92;
    float swingfz_kd = 200;
    
    float swinghx_kp = 120;
    float swinghy_kp = 110;
    float swinghz_kp = 160;
    
    float swinghx_kd = 120;
    float swinghy_kd = 92;
    float swinghz_kd = 200;
    Matrix<float, 6, 6>jacobian_swingMatrix;
    if (swingFlag == 0) 
    {
        legCmdPos(1,0) = -0.03 + x;
        legCmdPos(1,1) = -5.95758e-06  + y - legPresentPos(3,1) ;
        legCmdPos(1,2) = -0.64 + z;
        legCmdPos(2,0) = -0.03 + x;
        legCmdPos(2,1) = -5.95758e-06  + y - legPresentPos(3,1);
        legCmdPos(2,2) = -0.64 + z;
        legCmdVel(0) = vx;
        legCmdVel(1) = vy;
        legCmdVel(2) = vz;
        legCmdVel(3) = vx;
        legCmdVel(4) = vy;
        legCmdVel(5) = vz;
        jacobian_swingMatrix.block(0,0,3,3) << jacobian(1 ,0), jacobian(1 ,1), jacobian(1 ,2),
                                               jacobian(1 ,3), jacobian(1 ,4), jacobian(1 ,5),
                                               jacobian(1 ,6), jacobian(1 ,7), jacobian(1 ,8);
        jacobian_swingMatrix.block(3,3,3,3) << jacobian(2 ,0), jacobian(2 ,1), jacobian(2 ,2),
                                               jacobian(2 ,3), jacobian(2 ,4), jacobian(2 ,5),
                                               jacobian(2 ,6), jacobian(2 ,7), jacobian(2 ,8);
        jacobian_swingMatrix.block(0,3,3,3) = MatrixXf::Zero(3, 3);
        jacobian_swingMatrix.block(3,0,3,3) = MatrixXf::Zero(3, 3);                                       
        Vector<float, 6> temp_jointPresentVel;
        temp_jointPresentVel = jointPresentVel.segment(3, 6);
        legPresentVel = jacobian_swingMatrix * temp_jointPresentVel;
        Vector<float, 6> temp_forceswing;
        temp_forceswing(0) = swingfx_kp * (legCmdPos(1,0)- legPresentPos(1,0)) - swingfx_kd * (legPresentVel(0) - legCmdVel(0));
        temp_forceswing(1) = swingfy_kp * (legCmdPos(1,1)- legPresentPos(1,1)) - swingfy_kd * (legPresentVel(1) - legCmdVel(1));
        temp_forceswing(2) = swingfz_kp * (legCmdPos(1,2)- legPresentPos(1,2)) - swingfz_kd * (legPresentVel(2) - legCmdVel(2));
        temp_forceswing(3) = swinghx_kp * (legCmdPos(2,0)- legPresentPos(2,0)) - swinghx_kd * (legPresentVel(3) - legCmdVel(3));
        temp_forceswing(4) = swinghy_kp * (legCmdPos(2,1)- legPresentPos(2,1)) - swinghy_kd * (legPresentVel(4) - legCmdVel(4));
        temp_forceswing(5) = swinghz_kp * (legCmdPos(2,2)- legPresentPos(2,2)) - swinghz_kd * (legPresentVel(5) - legCmdVel(5));    
        jacobian_torque.segment(3, 6) = jacobian_swingMatrix.transpose() * temp_forceswing; 
    }
    else
    { 
        legCmdPos(0,0) = -0.03 + x;
        legCmdPos(0,1) = -5.95758e-06  + y - legPresentPos(2,1);
        legCmdPos(0,2) = -0.64 + z;
        legCmdPos(3,0) = -0.03 + x;
        legCmdPos(3,1) = -5.95758e-06  + y - legPresentPos(2,1);
        legCmdPos(3,2) = -0.64 + z;
        legCmdVel(0) = vx;
        legCmdVel(1) = vy;
        legCmdVel(2) = vz;
        legCmdVel(3) = vx;
        legCmdVel(4) = vy;
        legCmdVel(5) = vz;
        jacobian_swingMatrix.block(0,0,3,3) << jacobian(0 ,0), jacobian(0 ,1), jacobian(0 ,2),
                                               jacobian(0 ,3), jacobian(0 ,4), jacobian(0 ,5),
                                               jacobian(0 ,6), jacobian(0 ,7), jacobian(0 ,8);
        jacobian_swingMatrix.block(3,3,3,3) << jacobian(3 ,0), jacobian(3 ,1), jacobian(3 ,2),
                                               jacobian(3 ,3), jacobian(3 ,4), jacobian(3 ,5),
                                               jacobian(3 ,6), jacobian(3 ,7), jacobian(3 ,8);
        jacobian_swingMatrix.block(0,3,3,3) = MatrixXf::Zero(3, 3);
        jacobian_swingMatrix.block(3,0,3,3) = MatrixXf::Zero(3, 3); 
        Vector<float, 6> temp_jointPresentVel; 
        temp_jointPresentVel.head(3) = jointPresentVel.head(3);
        temp_jointPresentVel.tail(3) = jointPresentVel.tail(3);
        legPresentVel = jacobian_swingMatrix * temp_jointPresentVel;
        Vector<float, 6> temp_forceswing;
        temp_forceswing(0) = swingfx_kp * (legCmdPos(0,0)- legPresentPos(0,0)) - swingfx_kd * (legPresentVel(0) - legCmdVel(0));
        temp_forceswing(1) = swingfy_kp * (legCmdPos(0,1)- legPresentPos(0,1)) - swingfy_kd * (legPresentVel(1) - legCmdVel(1));
        temp_forceswing(2) = swingfz_kp * (legCmdPos(0,2)- legPresentPos(0,2)) - swingfz_kd * (legPresentVel(2) - legCmdVel(2));
        temp_forceswing(3) = swinghx_kp * (legCmdPos(3,0)- legPresentPos(3,0)) - swinghx_kd * (legPresentVel(3) - legCmdVel(3));
        temp_forceswing(4) = swinghy_kp * (legCmdPos(3,1)- legPresentPos(3,1)) - swinghy_kd * (legPresentVel(4) - legCmdVel(4));
        temp_forceswing(5) = swinghz_kp * (legCmdPos(3,2)- legPresentPos(3,2)) - swinghz_kd * (legPresentVel(5) - legCmdVel(5));
        Vector<float, 6> temp_torqueswing;       
        temp_torqueswing = jacobian_swingMatrix.transpose() * temp_forceswing;
        jacobian_torque.head(3) = temp_torqueswing.head(3);
        jacobian_torque.tail(3) = temp_torqueswing.tail(3);
    }
    
    fly_time += timePeriod;
    if (fly_time > T + 0.005)
    {
        fly_time = 0.0;
        if (swingFlag == 0)
        swingFlag = 1;
        else 
        swingFlag = 0;
    }
    
}

void MotionControl::inverseKinematics()
{
    float jo_ang[4][3] = {0};
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
        float theta2 = - acos ((pow(( sin(theta0) * y - cos(theta0) * z), 2)  + pow(x, 2) - pow(L1, 2) - pow(L2, 2)) / ( 2 * L1 * L2)) ; 
        float cos_theta1 = ((sin (theta2) * y - cos (theta0) * z) * (L1 + L2 * cos (theta2)) - x * L2 * sin(theta2)) / (pow((sin (theta2) * y - cos (theta0) * z), 2) + pow(x, 2));
        float sin_theta1 = ((L1 + L2 * cos (theta2)) * x + (sin (theta2) * y - cos (theta0) * z) * L2 * sin(theta2)) / (-pow((sin (theta2) * y - cos (theta0) * z), 2) - pow(x, 2));
        float theta1 = atan (sin_theta1/ cos_theta1);
        jo_ang[leg_num][0] = theta0;
        jo_ang[leg_num][1] = theta1;
        jo_ang[leg_num][2] = theta2;

    }
    
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

    for(int leg_nums = 0; leg_nums < 4; leg_nums++)
    {
        legPresentPos(leg_nums,0) = -L1 * sin(joint_pres_pos[leg_nums][1]) - L2 * sin(joint_pres_pos[leg_nums][1] + joint_pres_pos[leg_nums][2]);
        legPresentPos(leg_nums,1) = L1 * sin(joint_pres_pos[leg_nums][0]) * cos(joint_pres_pos[leg_nums][1]) + L2 * sin(joint_pres_pos[leg_nums][0]) * cos(joint_pres_pos[leg_nums][1] + joint_pres_pos[leg_nums][2]);
        legPresentPos(leg_nums,2) = -L1 * cos(joint_pres_pos[leg_nums][0]) * cos(joint_pres_pos[leg_nums][1]) - L2 * cos(joint_pres_pos[leg_nums][0]) * cos(joint_pres_pos[leg_nums][1] + joint_pres_pos[leg_nums][2]);
         
        leg2CoMPrePos(leg_nums,0) = shoulderPos(leg_nums,0) + legPresentPos(leg_nums,0);
        leg2CoMPrePos(leg_nums,1) = shoulderPos(leg_nums,1) + legPresentPos(leg_nums,1);
        leg2CoMPrePos(leg_nums,2) = legPresentPos(leg_nums,2);  
    }
}
void MotionControl::jacobians()
{
    jacobian(0 ,0) = 0;
    jacobian(0 ,1) =  -L1 * cos(jointPresentPos(1)) -L2 * cos(jointPresentPos(1) + jointPresentPos(2));
    jacobian(0 ,2) =  -L2 * cos(jointPresentPos(1) + jointPresentPos(2));
    jacobian(0 ,3) = L1 * cos(jointPresentPos(0)) * cos(jointPresentPos(1)) + L2 * cos(jointPresentPos(0)) * cos(jointPresentPos(1) + jointPresentPos(2));
    jacobian(0 ,4) = -L1 * sin(jointPresentPos(0)) * sin(jointPresentPos(1)) - L2 * sin(jointPresentPos(0)) * sin(jointPresentPos(1) + jointPresentPos(2));
    jacobian(0 ,5) = -L2 * sin(jointPresentPos(0)) * sin(jointPresentPos(1) + jointPresentPos(2));
    jacobian(0 ,6) = L1 * sin(jointPresentPos(0)) * cos(jointPresentPos(1)) + L2 * sin(jointPresentPos(0)) * cos(jointPresentPos(1) + jointPresentPos(2));
    jacobian(0 ,7) = L1 * cos(jointPresentPos(0)) * sin(jointPresentPos(1)) + L2 * cos(jointPresentPos(0)) * sin(jointPresentPos(1) + jointPresentPos(2));
    jacobian(0 ,8) = L2 * cos(jointPresentPos(0)) * sin(jointPresentPos(1) + jointPresentPos(2));

    jacobian(1 ,0) = 0;
    jacobian(1 ,1) =  -L1 * cos(jointPresentPos(4)) -L2 * cos(jointPresentPos(4) + jointPresentPos(5));
    jacobian(1 ,2) =  -L2 * cos(jointPresentPos(4) + jointPresentPos(5));
    jacobian(1 ,3) = L1 * cos(jointPresentPos(3)) * cos(jointPresentPos(4)) + L2 * cos(jointPresentPos(3)) * cos(jointPresentPos(4) + jointPresentPos(5));
    jacobian(1 ,4) = -L1 * sin(jointPresentPos(3)) * sin(jointPresentPos(4)) - L2 * sin(jointPresentPos(3)) * sin(jointPresentPos(4) + jointPresentPos(5));
    jacobian(1 ,5) = -L2 * sin(jointPresentPos(3)) * sin(jointPresentPos(4) + jointPresentPos(5));
    jacobian(1 ,6) = L1 * sin(jointPresentPos(3)) * cos(jointPresentPos(4)) + L2 * sin(jointPresentPos(3)) * cos(jointPresentPos(4) + jointPresentPos(5));
    jacobian(1 ,7) = L1 * cos(jointPresentPos(3)) * sin(jointPresentPos(4)) + L2 * cos(jointPresentPos(3)) * sin(jointPresentPos(4) + jointPresentPos(5));
    jacobian(1 ,8) = L2 * cos(jointPresentPos(3)) * sin(jointPresentPos(4) + jointPresentPos(5));

    jacobian(2, 0) = 0;
    jacobian(2 ,1) =  -L1 * cos(jointPresentPos(7)) -L2 * cos(jointPresentPos(7) + jointPresentPos(8));
    jacobian(2 ,2) =  -L2 * cos(jointPresentPos(7) + jointPresentPos(8));
    jacobian(2 ,3) = L1 * cos(jointPresentPos(6)) * cos(jointPresentPos(7)) + L2 * cos(jointPresentPos(6)) * cos(jointPresentPos(7) + jointPresentPos(8));
    jacobian(2 ,4) = -L1 * sin(jointPresentPos(6)) * sin(jointPresentPos(7)) - L2 * sin(jointPresentPos(6)) * sin(jointPresentPos(7) + jointPresentPos(8));
    jacobian(2 ,5) = -L2 * sin(jointPresentPos(6)) * sin(jointPresentPos(7) + jointPresentPos(8));
    jacobian(2 ,6) = L1 * sin(jointPresentPos(6)) * cos(jointPresentPos(7)) + L2 * sin(jointPresentPos(6)) * cos(jointPresentPos(7) + jointPresentPos(8));
    jacobian(2 ,7) = L1 * cos(jointPresentPos(6)) * sin(jointPresentPos(7)) + L2 * cos(jointPresentPos(6)) * sin(jointPresentPos(7) + jointPresentPos(8));
    jacobian(2 ,8) = L2 * cos(jointPresentPos(6)) * sin(jointPresentPos(7) + jointPresentPos(8));

    jacobian(3 ,0) = 0;
    jacobian(3 ,1) =  -L1 * cos(jointPresentPos(10)) -L2 * cos(jointPresentPos(10) + jointPresentPos(11));
    jacobian(3 ,2) =  -L2 * cos(jointPresentPos(10) + jointPresentPos(11));
    jacobian(3 ,3) = L1 * cos(jointPresentPos(9)) * cos(jointPresentPos(10)) + L2 * cos(jointPresentPos(9)) * cos(jointPresentPos(10) + jointPresentPos(11));
    jacobian(3 ,4) = -L1 * sin(jointPresentPos(9)) * sin(jointPresentPos(10)) - L2 * sin(jointPresentPos(9)) * sin(jointPresentPos(10) + jointPresentPos(11));
    jacobian(3 ,5) = -L2 * sin(jointPresentPos(9)) * sin(jointPresentPos(10) + jointPresentPos(11));
    jacobian(3 ,6) = L1 * sin(jointPresentPos(9)) * cos(jointPresentPos(10)) + L2 * sin(jointPresentPos(9)) * cos(jointPresentPos(10) + jointPresentPos(11));
    jacobian(3 ,7) = L1 * cos(jointPresentPos(9)) * sin(jointPresentPos(10)) + L2 * cos(jointPresentPos(9)) * sin(jointPresentPos(10) + jointPresentPos(11));
    jacobian(3 ,8) = L2 * cos(jointPresentPos(9)) * sin(jointPresentPos(10) + jointPresentPos(11));
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 9; j++)
        {
            if (jacobian(i, j) < -10 || jacobian (i, j) > 10)
            {
                jacobian(i ,j) = 0.0;
                cout << "jacobian"<<endl;
            }
        }        
     
    }
}
