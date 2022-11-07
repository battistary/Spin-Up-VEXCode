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
extern motor_group intake;
extern motor flywheel;
extern motor rollerRoller;
extern digital_out magazinePiston;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );