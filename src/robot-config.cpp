#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// VEXcode device constructors
controller controller1 = controller(primary);
controller controller2 = controller(partner);

motor driveLeftFront = motor(PORT11, ratio6_1, true);
motor driveLeftCenter = motor(PORT4, ratio6_1, true);
motor driveLeftBack = motor(PORT12, ratio6_1, true);
motor driveRightFront = motor(PORT1, ratio6_1, false);
motor driveRightCenter = motor(PORT5, ratio6_1, false);
motor driveRightBack = motor(PORT2, ratio6_1, false);
motor intake = motor(PORT3, ratio6_1, false);
motor flywheel = motor(PORT15, ratio6_1, true);

encoder leftEncoder = encoder(Brain.ThreeWirePort.A);
encoder rightEncoder = encoder(Brain.ThreeWirePort.C);
encoder backEncoder = encoder(Brain.ThreeWirePort.E);

digital_out stringLauncher = digital_out(Brain.ThreeWirePort.G);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}