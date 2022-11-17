using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor frontLeft;
extern motor backLeft;
extern motor frontRight;
extern motor backRight;
extern controller Controller1;
extern encoder leftEncoder;
extern encoder rightEncoder;
extern encoder backEncoder;
extern motor flywheel;
extern controller Controller2;
extern digital_out StringLauncherG;
extern motor intake;
extern motor leftMiddle;
extern motor rightMiddle;
extern gps GPS;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );