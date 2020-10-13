#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;
//motors
motor FR = motor(PORT11, ratio18_1, true);
motor FL = motor(PORT20, ratio18_1, false);
motor BL = motor(PORT19, ratio18_1, false);
motor BR = motor(PORT2, ratio18_1, true);
motor RightRoller = motor(PORT10, ratio18_1, true);
motor LeftRoller = motor(PORT1, ratio18_1, false);
motor Conveyor1 = motor(PORT9, ratio6_1, true);
motor Conveyor2 = motor(PORT3, ratio6_1, false);
controller Controller1 = controller(primary);

//sensors
inertial inertial_gyro = inertial(PORT5);
encoder leftTracker = encoder(Brain.ThreeWirePort.E);
encoder rightTracker = encoder(Brain.ThreeWirePort.F);
encoder horizontalTracker = encoder(Brain.ThreeWirePort.E);
line ballDetector1 = line(Brain.ThreeWirePort.H);
line ballDetector2 = line(Brain.ThreeWirePort.A);

void vexcodeInit(void) {
  // Nothing to initialize
}