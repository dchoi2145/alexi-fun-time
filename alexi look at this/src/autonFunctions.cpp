#include "vex.h"

#include "autonFunctions.h"

#include "driverFunctions.h"

using namespace vex;

void inertialCalibration(){
  inertial_gyro.calibrate();
   while (inertial_gyro.isCalibrating()){
     wait(100, msec);
   }
   
}

void reset(){
  BL.resetRotation();
  BR.resetRotation();
  FL.resetRotation();
  FR.resetRotation();
  leftTracker.resetRotation();
  rightTracker.resetRotation();
  horizontalTracker.resetRotation();
  inertial_gyro.resetRotation();
}
//constants
double kP = 0;
double kI = 0;
double kD = 0;

//auton settings
int target = -100;

//variables
int error; //SensorValue - target : PositionalValue
int previousError = 0; //Position 20 ms ago
int derivative; //Error - previousError : Speed
int totalError = 0; 
bool enableDrivePID = true;

int drivePID(int target) {

  while(enableDrivePID) {
     //get position of each tracker
     int leftTrackerPos = rightTracker.position(degrees); 
     int rightTrackerPos = rightTracker.position(degrees);
     
     //get position of both trackers
     int averagePos = (leftTrackerPos + rightTrackerPos)/2; //get position of both trackers
     
     //proportional
     error = averagePos - target;
     
     //derivative
     derivative = error - previousError;
     
     //integral
     totalError += error;

     double motorPower = error * kP + derivative * kD * totalError * kI;

     FL.spin(forward, motorPower, velocityUnits::pct);
     BL.spin(forward, motorPower, velocityUnits::pct);
     FR.spin(forward, motorPower, velocityUnits::pct);
     BR.spin(forward, motorPower, velocityUnits::pct);

     previousError = error;
     task::sleep(20);
  }

  return 1;
}

void moveForward(){
  drivePID(-100);
}

  
