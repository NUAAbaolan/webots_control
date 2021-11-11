// File:          my_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
// #include <webots/Robot.hpp>
// #include <motiondefine.hpp>
// #include <webots/Robot.hpp>
// #include <webots/Device.hpp>
// #include <webots/Motor.hpp>
// #include <webots/PositionSensor.hpp>
#include <motionControl.cpp>
#include <ctime>
#include <webots/Keyboard.hpp>
// #include <webots/TouchSensor.hpp>
// #include <webots/InertialUnit.hpp>
#define PI 3.1415926

// #include <motionControl.cpp>
// All the webots classes are defined in the "webots" namespace
using namespace webots;
MotionControl mc;
  
int main(int argc, char **argv) {
    Robot *robot = new Robot(); 
    Keyboard kb;
    kb.enable(1000);
    int key;
    float timePeriod = 0.01;
    float timeForGaitPeriod = 0.60;
    Matrix<float, 4, 2> timeForStancePhase;
    timeForStancePhase << 0, 0.24, 0.30, 0.54, 0.30, 0.54, 0, 0.24;
    mc.MotionContr(timePeriod, timeForGaitPeriod, timeForStancePhase,robot);
    Camera *camera;
    camera=robot->getCamera("camera");
    camera->enable(TIME_STEP);
    
    int timeStep = (int)robot->getBasicTimeStep();
    
    Matrix<float, 4, 3> initPos; 
    initPos<<  0.0261167 ,-5.95758e-06  ,  -0.640955, 0.026106, -6.00657e-06 ,   -0.640965, 0.026248, -5.66663e-06 ,  -0.64084, 0.0262556 , 4.52107e-06,  -0.640832;
    mc.setInitPos(initPos);
    Vector<float, 3> tCV;
    tCV<< 0.0, 0.0, 0.0;
    //w:87,s:83,a:65,d:68
    mc.swingFlag = 0;
    mc.setCoMVel(tCV); 
    char StateVal;
    StateVal = mc.stateval[0];
  while (robot->step(timeStep) != -1) 
  {
    // clock_t startTime,endTime;
    // startTime = clock();//计时开始
    switch(StateVal)
    {
      case 'a'://
         mc.State_start();
         if (mc.Start_time >= 0.20)
         {   
           StateVal = mc.stateval[1];
           // mc.Start_time = 0.0;
           mc.fly_time = 0.0;
         }
         cout << "a"<<endl;
         break;
      case 'b':
          key = kb.getKey();
          if (key == 87)
          {
            tCV(0) = tCV(0) + 0.01; 
          }
          else if (key == 83)
          {
            tCV(0) = tCV(0) - 0.01;
          }
          else if (key == 65)
          {
            tCV(1) = tCV(1) + 0.01; 
          }
          else if (key == 68)
          {
            tCV(1) = tCV(1) - 0.01; 
          }
          else
          {
            tCV = tCV;
          }
          // cout << key << endl;
         mc.setCoMVel(tCV);
         mc.Sensor_update();
         mc.forwardKinematics();
         mc.state_judgement();
         mc.jacobians();
         mc.stance_VMC();
         mc.swing_VMC();
         mc.inverseKinematics();
         mc.Setjoint();
         cout << "v:   "<<tCV(0)<<endl;
         // cout << "leg:   "<< mc.legPresentPos << endl;
         
         // switch(mc.state_val)
         // {
           // case 1:
              // if (mc.swingFlag == 0)
              // {
                // //23摆动，14支撑
                
              // }
              // else
              // {
                // //14摆动，23支撑
                
              // }
           // case 2:
           
         // }
         break;
      default:
          
         break;
       // case two_leg_state1      
    }
    // endTime = clock();//计时结束
    // cout << "The run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
  };

delete robot;
return 0;
}
