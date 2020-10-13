/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Daquan Fisher                                             */
/*    Created:      September 2020 - April 2021                               */
/*    Description:  2145Z Competition Code                                    */
/*                                                                            */
/*----------------------------------------------------------------------------*/



#include "vex.h"

#include "autonFunctions.h"

#include "driverFunctions.h"

using namespace vex;


competition Competition;


// Initializing Robot Configuration
void pre_auton(void) {
  vexcodeInit();
  inertialCalibration();
}


//auton goes in here
void autonomous(void) {
    moveForward();
  }

// User control code 
void usercontrol(void) {
    
  
  while (true) {
    task p = task(chassisMovement);
    task i = task(Intake);
    printLineValue1();
    printLineValue2();
  }

       wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
    }
  
  

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
