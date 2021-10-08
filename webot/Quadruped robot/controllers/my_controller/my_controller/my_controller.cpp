// File:          my_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
// #include <webots/Robot.hpp>
#include <motiondefine.h>
#include <webots/Robot.hpp>
#include <webots/Device.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <motionControl.cpp>
#define PI 3.1415926
#define TIME_STEP 64
// #include <motionControl.cpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;
 
  
// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
  MotionControl mc;
  
  
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot(); 
  
  float timePeriod = 0.01;
  float timeForGaitPeriod = 0.49;
  Matrix<float, 4, 2> timeForStancePhase;
  timeForStancePhase << 0, 0.24, 0.25, 0.49, 0.25, 0.49, 0, 0.24;
  mc.timeForStancePhase = timeForStancePhase;
  mc.MotionContr(timePeriod, timeForGaitPeriod, timeForStancePhase); 
 
  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
   Matrix<float, 4, 3> initPos; 
   // initPos<< 0.0259623, -4.93015e-06, -0.641027, 0.0259617, 1.29909e-06, -0.641112,  0.0259029, 2.28772e-06, -0.64105, 0.0259022, 2.4387e-06, -0.64105;
   initPos<< 0.0259623,-4.93015e-06, -0.641027, 0.0259617, -4.93069e-06 ,   -0.64102 , 0.0260543 ,-4.72215e-06,    -0.640945, 0.0260546,  4.60696e-06,    -0.640945;
   Vector<float, 3> tCV;
   tCV<< 0.0, 0.0, 0.0;
   mc.setInitPos(initPos);
   mc.setCoMVel(tCV);
   
   PositionSensor *LF_joint1 = robot->getPositionSensor("LFL0_position sensor");
   PositionSensor *LF_joint2 = robot->getPositionSensor("LFL1_position sensor");
   PositionSensor *LF_joint3 = robot->getPositionSensor("LFL2_position sensor");
   PositionSensor *RF_joint1 = robot->getPositionSensor("RFL0_position sensor");
   PositionSensor *RF_joint2 = robot->getPositionSensor("RFL1_position sensor");
   PositionSensor *RF_joint3 = robot->getPositionSensor("RFL2_position sensor");
   PositionSensor *LH_joint1 = robot->getPositionSensor("LBL0_position sensor");
   PositionSensor *LH_joint2 = robot->getPositionSensor("LBL1_position sensor");
   PositionSensor *LH_joint3 = robot->getPositionSensor("LBL2_position sensor");
   PositionSensor *RH_joint1 = robot->getPositionSensor("RBL0_position sensor");
   PositionSensor *RH_joint2 = robot->getPositionSensor("RBL1_position sensor");
   PositionSensor *RH_joint3 = robot->getPositionSensor("RBL2_position sensor");
   
   LF_joint1->enable(TIME_STEP);
   LF_joint2->enable(TIME_STEP);
   LF_joint3->enable(TIME_STEP);
   RF_joint1->enable(TIME_STEP);
   RF_joint2->enable(TIME_STEP);
   RF_joint3->enable(TIME_STEP);
   LH_joint1->enable(TIME_STEP);
   LH_joint2->enable(TIME_STEP); 
   LH_joint3->enable(TIME_STEP);
   RH_joint1->enable(TIME_STEP);
   RH_joint2->enable(TIME_STEP);
   RH_joint3->enable(TIME_STEP);
    
   Motor *LF_Motor1 = robot->getMotor("LFL0_rotational motor");
   Motor *LF_Motor2 = robot->getMotor("LFL1_rotational motor");
   Motor *LF_Motor3 = robot->getMotor("LFL2_rotational motor");
   Motor *RF_Motor1 = robot->getMotor("RFL0_rotational motor");
   Motor *RF_Motor2 = robot->getMotor("RFL1_rotational motor");
   Motor *RF_Motor3 = robot->getMotor("RFL2_rotational motor");
   Motor *LH_Motor1 = robot->getMotor("LBL0_rotational motor");
   Motor *LH_Motor2 = robot->getMotor("LBL1_rotational motor");
   Motor *LH_Motor3 = robot->getMotor("LBL2_rotational motor");
   Motor *RH_Motor1 = robot->getMotor("RBL0_rotational motor");
   Motor *RH_Motor2 = robot->getMotor("RBL1_rotational motor");
   Motor *RH_Motor3 = robot->getMotor("RBL2_rotational motor");
   
   struct timeval startTime, endTime; 
   
   int time = 1;
  while (robot->step(timeStep) != -1) 
  {
    mc.jointPresentPos(0) = LF_joint1->getValue();
    mc.jointPresentPos(1) = LF_joint2->getValue();
    mc.jointPresentPos(2) = LF_joint3->getValue();
    mc.jointPresentPos(3) = RF_joint1->getValue();
    mc.jointPresentPos(4) = RF_joint2->getValue();
    mc.jointPresentPos(5) = RF_joint3->getValue();
    mc.jointPresentPos(6) = LH_joint1->getValue();
    mc.jointPresentPos(7) = LH_joint2->getValue();
    mc.jointPresentPos(8) = LH_joint3->getValue();
    mc.jointPresentPos(9) = RH_joint1->getValue();
    mc.jointPresentPos(10) = RH_joint2->getValue();
    mc.jointPresentPos(11) = RH_joint3->getValue();
    mc.forwardKinematics();
    cout << "leg" << "  " << mc.legCmdPos << endl;
    for (int i = 0; i < 12 ;i++)
    {
        mc.jointCmdPos[i] = mc.jointPresentPos(i);
    }
    LF_Motor1->setPosition(mc.jointCmdPos[0]);
    LF_Motor2->setPosition(mc.jointCmdPos[1]);
    LF_Motor3->setPosition(mc.jointCmdPos[2]);
    
    RF_Motor1->setPosition(mc.jointCmdPos[3]);
    RF_Motor2->setPosition(mc.jointCmdPos[4]);
    RF_Motor3->setPosition(mc.jointCmdPos[5]);
    
    LH_Motor1->setPosition(mc.jointCmdPos[6]);
    LH_Motor2->setPosition(mc.jointCmdPos[7]);
    LH_Motor3->setPosition(mc.jointCmdPos[8]);
    
    RH_Motor1->setPosition(mc.jointCmdPos[9]);
    RH_Motor2->setPosition(mc.jointCmdPos[10]);
    RH_Motor3->setPosition(mc.jointCmdPos[11]);
    if (time == 200)
        break;
    time += 1;
  }
  
  
  while (robot->step(timeStep) != -1) 
  {
  
    gettimeofday(&startTime,NULL);
    //get present joint
    Vector<float, 12> jointLastPos;
    for (int i = 0; i < 12; i++)
    {
        jointLastPos(i) = mc.jointPresentPos(i);
    }
    mc.jointPresentPos(0) = LF_joint1->getValue();
    mc.jointPresentPos(1) = LF_joint2->getValue();
    mc.jointPresentPos(2) = LF_joint3->getValue();
    mc.jointPresentPos(3) = RF_joint1->getValue();
    mc.jointPresentPos(4) = RF_joint2->getValue();
    mc.jointPresentPos(5) = RF_joint3->getValue();
    mc.jointPresentPos(6) = LH_joint1->getValue();
    mc.jointPresentPos(7) = LH_joint2->getValue();
    mc.jointPresentPos(8) = LH_joint3->getValue();
    mc.jointPresentPos(9) = RH_joint1->getValue();
    mc.jointPresentPos(10) = RH_joint2->getValue();
    mc.jointPresentPos(11) = RH_joint3->getValue();
    for (int i = 0; i < 12; i++)
    {
         mc.jointPresentVel(i) = (mc.jointPresentPos(i) - jointLastPos(i))/ timePeriod;
    } 
        
    // std::cout<<"joint vec"<<"  "<<mc.jointPresentVel.transpose() << std::endl; 

    
    mc.forwardKinematics();
    // mc.inverseKinematics();
    mc.initFlag = true;
    mc.nextStep();
    mc.inverseKinematics();
    // mc.pid();
    
    // Vector<float, 12> temp_torque; 
    // for (int i = 0; i < 12; i++)
    // {
      // temp_torque(i) = mc.pid_motortorque[i];
    // }
    // cout << "present torque" << "  " <<temp_torque.transpose() << endl;  
    // std::cout<<"command joint"<< " "<<mc.jointCmdPos[0]<<" "<< mc.jointCmdPos[1]<<" "<<mc.jointCmdPos[2]<< " "<<mc.jointCmdPos[3]<<" "<< mc.jointCmdPos[4]<<" "<<mc.jointCmdPos[5]<< " "<<mc.jointCmdPos[6]<<" "<< mc.jointCmdPos[7]<<" "<<mc.jointCmdPos[8]<< " "<<mc.jointCmdPos[9]<<" "<< mc.jointCmdPos[10]<<" "<<mc.jointCmdPos[11]<< std::endl;
    
    
    //set position
    LF_Motor1->setPosition(mc.jointCmdPos[0]);
    LF_Motor2->setPosition(mc.jointCmdPos[1]);
    LF_Motor3->setPosition(mc.jointCmdPos[2]);
    
    RH_Motor1->setPosition(mc.jointCmdPos[9]);
    RH_Motor2->setPosition(mc.jointCmdPos[10]);
    RH_Motor3->setPosition(mc.jointCmdPos[11]);
    
    RF_Motor1->setPosition(mc.jointCmdPos[3]);
    RF_Motor2->setPosition(mc.jointCmdPos[4]);
    RF_Motor3->setPosition(mc.jointCmdPos[5]);
    
    LH_Motor1->setPosition(mc.jointCmdPos[6]);
    LH_Motor2->setPosition(mc.jointCmdPos[7]);
    LH_Motor3->setPosition(mc.jointCmdPos[8]);
    
    
    
    //set torque
    // LF_Motor1->setTorque(mc.pid_motortorque[0]);
    // LF_Motor2->setTorque(mc.pid_motortorque[1]);
    // LF_Motor3->setTorque(mc.pid_motortorque[2]);
    
    // RF_Motor1->setTorque(mc.pid_motortorque[3]);
    // RF_Motor2->setTorque(mc.pid_motortorque[4]);
    // RF_Motor3->setTorque(mc.pid_motortorque[5]);

    // LH_Motor1->setTorque(mc.pid_motortorque[6]);
    // LH_Motor2->setTorque(mc.pid_motortorque[7]);
    // LH_Motor3->setTorque(mc.pid_motortorque[8]);
    
    // RH_Motor1->setTorque(mc.pid_motortorque[9]);
    // RH_Motor2->setTorque(mc.pid_motortorque[10]);
    // RH_Motor3->setTorque(mc.pid_motortorque[11]);

    gettimeofday(&endTime,NULL);
    double timeUse = 1000000*(endTime.tv_sec - startTime.tv_sec) + endTime.tv_usec - startTime.tv_usec;
    cout <<"time" << timeUse<<endl;
    // usleep(1000 - timeUse);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}