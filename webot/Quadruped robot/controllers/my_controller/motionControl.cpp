#include <motion.hpp>
#define PI 3.1415926

// using namespace std;
// using namespace Eigen;

void MotionControl::getpresentJoint()
{
  double value;
  WbDeviceTag sensor = wb_robot_get_device("LFL0_position sensor");
  value = wb_position_sensor_get_value(sensor);
  cout << "Sensor value is: " << value << endl;
}