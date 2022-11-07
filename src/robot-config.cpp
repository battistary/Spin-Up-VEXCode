#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor frontLeft = motor(PORT11, ratio18_1, true);
motor backLeft = motor(PORT12, ratio18_1, true);
motor frontRight = motor(PORT1, ratio18_1, false);
motor backRight = motor(PORT2, ratio18_1, false);
controller Controller1 = controller(primary);
encoder leftEncoder = encoder(Brain.ThreeWirePort.A);
encoder rightEncoder = encoder(Brain.ThreeWirePort.C);
encoder backEncoder = encoder(Brain.ThreeWirePort.E);
motor intakeMotorA = motor(PORT3, ratio18_1, false);
motor intakeMotorB = motor(PORT4, ratio18_1, true);
motor_group intake = motor_group(intakeMotorA, intakeMotorB);
motor flywheel = motor(PORT15, ratio6_1, true);
motor rollerRoller = motor(PORT5, ratio18_1, false);
digital_out magazinePiston = digital_out(Brain.ThreeWirePort.H);

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