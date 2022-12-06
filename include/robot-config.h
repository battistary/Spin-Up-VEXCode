using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern controller Controller2;

extern motor driveLeftFront;
extern motor driveLeftCenter;
extern motor driveLeftBack;
extern motor driveRightFront;
extern motor driveRightCenter;
extern motor driveRightBack;
extern motor intake;
extern motor flywheel;

extern encoder leftEncoder;
extern encoder rightEncoder;
extern encoder backEncoder;

extern digital_out stringLauncher;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);