#include "vex.h"

#include "driverFunctions.h"

#include "autonFunctions.h"

using namespace vex;


double chassisMultiplier = 1;
//code for drive
int chassisMovement(){
  while(true) {
    double front_left = (double)(Controller1.Axis3.position(pct) + Controller1.Axis4.position(pct));
    double back_left = (double)(Controller1.Axis3.position(pct) - Controller1.Axis4.position(pct));
    double front_right = (double)(Controller1.Axis3.position(pct) - Controller1.Axis4.position(pct));
    double back_right = (double)(Controller1.Axis3.position(pct) + Controller1.Axis4.position(pct));

    double max_raw_sum = (double)(abs(Controller1.Axis3.position(pct)) + abs(Controller1.Axis4.position(pct)));

    double max_XYstick_value = (double)(std::max(abs(Controller1.Axis3.position(pct)), abs(Controller1.Axis4.position(pct))));

    if (max_raw_sum != 0) {
      front_left = front_left / max_raw_sum * max_XYstick_value;
      back_left = back_left / max_raw_sum * max_XYstick_value;
      front_right = front_right / max_raw_sum * max_XYstick_value;
      back_right = back_right / max_raw_sum * max_XYstick_value;
    }

      front_left = front_left + Controller1.Axis1.position(pct);
      back_left = back_left + Controller1.Axis1.position(pct);
      front_right = front_right - Controller1.Axis1.position(pct);
      back_right = back_right - Controller1.Axis1.position(pct);

    max_raw_sum = std::max(fabs(front_left), std::max(fabs(back_left), std::max(fabs(front_right), std::max(fabs(back_right), 100.0))));

    front_left = front_left / max_raw_sum * 100.0;
    back_left = back_left / max_raw_sum * 100.0;
    front_right = front_right / max_raw_sum * 100.0;
    back_right = back_right / max_raw_sum * 100.0;

    FL.spin(forward, front_left , velocityUnits::pct);
    FR.spin(forward, front_right , velocityUnits::pct);
    BL.spin(forward, back_left , velocityUnits::pct);
    BR.spin(forward, back_right , velocityUnits::pct);

    task::sleep(10);
  
  }
}
  //all intake control functions
  int Intake() {
  while (true) {

  int lineSensor1Value = ballDetector1.value(pct);

      if (Controller1.ButtonR1.pressing() && Controller1.ButtonR2.pressing()) {
        LeftRoller.spin(fwd, 100, pct);
        RightRoller.spin(fwd, 100, pct);
        Conveyor1.spin(fwd, 100, pct);
        Conveyor2.spin(fwd, 100, pct);
    }

      else if (Controller1.ButtonL1.pressing() && Controller1.ButtonL2.pressing()) {
        LeftRoller.spin(reverse, 100, pct);
        RightRoller.spin(reverse, 100, pct);
        Conveyor1.spin(reverse, 100, pct);
        Conveyor2.spin(reverse, 100, pct);
    }

      else if (Controller1.ButtonL1.pressing()) {
        Conveyor1.spin(fwd, 100, pct);
        Conveyor2.spin(fwd, 100, pct);
    }

      else if (Controller1.ButtonL2.pressing()) {
        Conveyor1.spin(reverse, 100, pct);
        Conveyor2.spin(reverse, 100, pct);
    }

      else if (lineSensor1Value <= 69){
        Conveyor1.spin(fwd, 100, pct);
        Conveyor2.spin(fwd, 100, pct);
    }

      /*else if(ballDetector2.value(pct) <= 70){
        lineSensor1Value = 100;
    }

      else if(ballDetector2.value(pct) > 70){
        lineSensor1Value = ballDetector1.value(pct);
    }*/

      else if (Controller1.ButtonR2.pressing()) {
        LeftRoller.spin(reverse, 100, pct);
        RightRoller.spin(reverse, 100, pct);
    }

      else if (Controller1.ButtonR1.pressing()) {
        LeftRoller.spin(fwd, 100, pct);
        RightRoller.spin(fwd, 100, pct);
    }

      else {
      LeftRoller.stop(coast);
      RightRoller.stop(coast);
      Conveyor1.stop(coast);
      Conveyor2.stop(coast);
    }
  
    task::sleep(10);
    }
  }
  //prints value of line sensor to screen
   void printLineValue1() {
      Brain.Screen.setCursor(1,1);
      Brain.Screen.print("Line Sensor1:");
      Brain.Screen.setCursor(1,14);
      Brain.Screen.print(ballDetector1.value(pct));

  }
  //prints value of line sensor to screen
   void printLineValue2(){
      Brain.Screen.setCursor(2,1);
      Brain.Screen.print("Line Sensor2:");
      Brain.Screen.setCursor(2,14);
      Brain.Screen.print(ballDetector2.value(pct));

   }
   
   
    