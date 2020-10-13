using namespace vex;

extern brain Brain;

extern motor FR; 
extern motor FL; 
extern motor BL; 
extern motor BR;
extern motor RightRoller; 
extern motor LeftRoller;
extern motor Conveyor1;
extern motor Conveyor2;
extern controller Controller1; 
extern inertial inertial_gyro;
extern encoder leftTracker;
extern encoder rightTracker;
extern encoder horizontalTracker;
extern line ballDetector1;
extern line ballDetector2;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
