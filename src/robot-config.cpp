#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor frontLeft = motor(PORT11, ratio6_1, false);
motor backLeft = motor(PORT12, ratio6_1, true);
motor frontRight = motor(PORT1, ratio6_1, false);
motor backRight = motor(PORT2, ratio6_1, true);
controller Controller1 = controller(primary);
encoder leftEncoder = encoder(Brain.ThreeWirePort.A);
encoder rightEncoder = encoder(Brain.ThreeWirePort.C);
encoder backEncoder = encoder(Brain.ThreeWirePort.E);
motor flywheel = motor(PORT15, ratio6_1, true);
controller Controller2 = controller(partner);
digital_out StringLauncherG = digital_out(Brain.ThreeWirePort.G);
motor intake = motor(PORT3, ratio6_1, false);
motor leftMiddle = motor(PORT4, ratio6_1, false);
motor rightMiddle = motor(PORT5, ratio6_1, false);
gps GPS = gps(PORT6, 0.00, 0.00, mm, 180);

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