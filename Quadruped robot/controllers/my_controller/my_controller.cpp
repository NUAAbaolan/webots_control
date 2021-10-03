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
  void getposition(Robot *robot);
  void setposition(Robot *robot);
  
  
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
   initPos<< 0.641163, 0, -0.0256602, 0.641163, 0, -0.0256602, 0.641163, 0, -0.0256602, 0.641163, 0, -0.0256602;
   Vector<float, 3> tCV;
   tCV<< 0.0, 0.0, 0.0;
   mc.setInitPos(initPos);
   float time = 0.0;
   
   
   
   
  while (robot->step(timeStep) != -1) {
  
    getposition(robot);
    for (int i = 0; i<12; i++)
    {
      mc.jointCmdPos[i] = mc.jointPresentPos(i);
    }
    // mc.setCoMVel(tCV);
    // mc.forwardKinematics();
    // mc.inverseKinematics();
    // mc.initFlag = true;
    // mc.nextStep();
    // mc.inverseKinematics();
    mc.jointCmdPos[0] = mc.jointCmdPos[0] + time * 0.1;
    time = time + 1;
    std::cout<<mc.jointPresentPos.transpose() << std::endl; 
    
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.
    
    setposition(robot);

    
    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}

void getposition(Robot *robot)
{

  PositionSensor *LF_joint1 = robot->getPositionSensor("LFL0_position sensor");
  LF_joint1->enable(TIME_STEP);
  mc.jointPresentPos(0) = LF_joint1->getValue();
  
  PositionSensor *LF_joint2 = robot->getPositionSensor("LFL1_position sensor");
  LF_joint2->enable(TIME_STEP);
  mc.jointPresentPos(1) = LF_joint2->getValue();
  
  PositionSensor *LF_joint3 = robot->getPositionSensor("LFL2_position sensor");
  LF_joint3->enable(TIME_STEP);
  mc.jointPresentPos(2) = LF_joint3->getValue();
  
  PositionSensor *RF_joint1 = robot->getPositionSensor("RFL0_position sensor");
  RF_joint1->enable(TIME_STEP);
  mc.jointPresentPos(3) = RF_joint1->getValue();
  
  PositionSensor *RF_joint2 = robot->getPositionSensor("RFL1_position sensor");
  RF_joint2->enable(TIME_STEP);
  mc.jointPresentPos(4) = RF_joint2->getValue();
  
  PositionSensor *RF_joint3 = robot->getPositionSensor("RFL2_position sensor");
  RF_joint3->enable(TIME_STEP);
  mc.jointPresentPos(5) = RF_joint3->getValue();
  
  PositionSensor *LH_joint1 = robot->getPositionSensor("LBL0_position sensor");
  LH_joint1->enable(TIME_STEP); 
  mc.jointPresentPos(6) = LH_joint1->getValue();
  
  PositionSensor *LH_joint2 = robot->getPositionSensor("LBL1_position sensor");
  LH_joint2->enable(TIME_STEP);
  mc.jointPresentPos(7) = LH_joint2->getValue();
  
  PositionSensor *LH_joint3 = robot->getPositionSensor("LBL2_position sensor");
  LH_joint3->enable(TIME_STEP);
  mc.jointPresentPos(8) = LH_joint3->getValue();
  
  PositionSensor *RH_joint1 = robot->getPositionSensor("RBL0_position sensor");
  RH_joint1->enable(TIME_STEP);
  mc.jointPresentPos(9) = RH_joint1->getValue();
  
  PositionSensor *RH_joint2 = robot->getPositionSensor("RBL1_position sensor");
  RH_joint2->enable(TIME_STEP);
  mc.jointPresentPos(10) = RH_joint2->getValue();
  
  PositionSensor *RH_joint3 = robot->getPositionSensor("RBL2_position sensor");
  RH_joint3->enable(TIME_STEP);
  mc.jointPresentPos(11) = RH_joint3->getValue();
  
  // while (robot->step(TIME_STEP) != -1) 
  // {
    // std::cout << mc.jointPresentPos.transpose() << std::endl; 
  // };
}
void setposition(Robot *robot)
{
  // for (int i = 0; i <12; i++)
  // {
    // mc.jointCmdPos[i] = 0.0;
  // }
  Motor *LF_Motor1 = robot->getMotor("LFL0_rotational motor");
  LF_Motor1->setPosition(mc.jointCmdPos[0]);
  Motor *LF_Motor2 = robot->getMotor("LFL1_rotational motor");
  LF_Motor2->setPosition(mc.jointCmdPos[1]);
  Motor *LF_Motor3 = robot->getMotor("LFL2_rotational motor");
  LF_Motor3->setPosition(mc.jointCmdPos[2]);
  Motor *RF_Motor1 = robot->getMotor("RFL0_rotational motor");
  RF_Motor1->setPosition(mc.jointCmdPos[3]);
  Motor *RF_Motor2 = robot->getMotor("RFL1_rotational motor");
  RF_Motor2->setPosition(mc.jointCmdPos[4]);
  Motor *RF_Motor3 = robot->getMotor("RFL2_rotational motor");
  RF_Motor3->setPosition(mc.jointCmdPos[5]);
  Motor *LH_Motor1 = robot->getMotor("LBL0_rotational motor");
  LH_Motor1->setPosition(mc.jointCmdPos[6]);
  Motor *LH_Motor2 = robot->getMotor("LBL1_rotational motor");
  LH_Motor2->setPosition(mc.jointCmdPos[7]);
  Motor *LH_Motor3 = robot->getMotor("LBL2_rotational motor");
  LH_Motor3->setPosition(mc.jointCmdPos[8]);
  Motor *RH_Motor1 = robot->getMotor("RBL0_rotational motor");
  RH_Motor1->setPosition(mc.jointCmdPos[9]);
  Motor *RH_Motor2 = robot->getMotor("RBL1_rotational motor");
  RH_Motor2->setPosition(mc.jointCmdPos[10]);
  Motor *RH_Motor3 = robot->getMotor("RBL2_rotational motor");
  RH_Motor3->setPosition(mc.jointCmdPos[11]); 
    
}