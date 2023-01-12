using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller controller1;
extern controller controller2;

extern motor driveLeftFront;
extern motor driveLeftCenter;
extern motor driveLeftBack;
extern motor driveRightFront;
extern motor driveRightCenter;
extern motor driveRightBack;
extern motor flywheel;
extern motor intake;

extern motor_group driveLeft;
extern motor_group driveRight;

extern encoder leftEncoder;
extern encoder rightEncoder;
extern encoder backEncoder;

extern inertial inertialSensor;

extern digital_out stringLauncher;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);