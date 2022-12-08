/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Ryan Battista and Shrikar Seshadri                        */
/*    Created:      Wed Jun 8 2022                                            */
/*    Description:  9909Y Yottabyte 2022-2023 VEX Spin Up Code                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// controller1          controller
// controller2          controller
// driveLeftFront       motor         11
// driveLeftCenter      motor         4
// driveLeftBack        motor         12
// driveRightFront      motor         1
// driveRightCenter     motor         5
// driveRightBack       motor         2
// intake               motor         18
// flywheel             motor         15
// driveLeft            motor_group
// driveRight           motor_group
// leftEncoder          encoder       A, B
// rightEncoder         encoder       C, D
// backEncoder          encoder       E, F
// stringLauncher       digital_out   G
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "logo.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  leftEncoder.setPosition(0, degrees);
  rightEncoder.setPosition(0, degrees);
  backEncoder.setPosition(0, degrees);

  driveLeftBack.setBrake(coast);
  driveLeftCenter.setBrake(coast);
  driveLeftFront.setBrake(coast);

  driveRightBack.setBrake(coast);
  driveRightCenter.setBrake(coast);
  driveRightFront.setBrake(coast);

  intake.setBrake(brake);
  flywheel.setBrake(coast);

  stringLauncher.set(0);
}

/*---------------------------------------------------------------------------*/
/*                            Baseline PD Controller                         */
/*---------------------------------------------------------------------------*/

// Settings - TUNE TO OUR ROBOT
double kP = 0.0;
double kD = 0.0;
double turnkP = 0.0;
double turnkD = 0.0;

// Autonomous settings
int desiredValue = 200;
int desiredTurnValue = 0;

int error; // SensorValue - DesiredValue : position
int prevError = 0; // error 20 milliseconds ago
int derivative; // error - prevError (speed)
int totalError = 0; //totalError = totalError + error (integral)

int turnError; //SensorValue - DesiredValue : position
int turnPrevError = 0; // error 20 milliseconds ago
int turnDerivative; // error - prevError (speed)
int turnTotalError = 0; //totalError = totalError + error (integral)

bool resetDriveSensors = false;
bool enabledrivePID = false;

int drivePID() {

  while(enabledrivePID){

    if (resetDriveSensors){
      resetDriveSensors = false;
      driveLeft.setPosition(0, degrees);
      driveRight.setPosition(0, degrees);
      leftEncoder.setPosition(0, degrees);
      rightEncoder.setPosition(0, degrees);
      backEncoder.setPosition(0, degrees);
    }

    // Get the positions of the shaft encoders
    int leftEncoderPosition = leftEncoder.position(degrees);
    int rightEncoderPosition = rightEncoder.position(degrees);

    /*******************************************/
    /*          Lateral Movement PID           */
    /*******************************************/
    
    // Get average of the two shaft encoders
    int averagePosition = (leftEncoderPosition + rightEncoderPosition) / 2;
    
    // Potential
    error = (desiredValue*49.8637150129) - averagePosition;

    // Derivative
    derivative = error - prevError;

    double LateralMotorPower = (error * kP + derivative * kD) / 12.0;

    /*******************************************/
    /*          Turning Movement PD            */
    /*******************************************/

    // Get average of the two motors
    int turnDifference = leftEncoderPosition - rightEncoderPosition;
    
    // Potential
    turnError = ((desiredTurnValue/360)*3.256*360) - turnDifference;

    // Derivative
    turnDerivative = turnError - turnPrevError;

    /////////////////////////////////////////////

    double TurnMotorPower = (turnError * turnkP + turnDerivative * turnkD) / 12.0;
    
    driveLeft.spin(forward, LateralMotorPower + TurnMotorPower, voltageUnits::volt);
    driveRight.spin(forward, LateralMotorPower - TurnMotorPower, voltageUnits::volt);

    prevError = error;
    turnPrevError = turnError;
    vex::task::sleep(20);
  }

return 0;
}

/*---------------------------------------------------------------------------*/
/*                             Driving Functions                             */
/*---------------------------------------------------------------------------*/

// Drive Forward 
void driveForward(double inches, rotationUnits degrees, double velocity, percentUnits pct) {
  driveLeft.setVelocity(velocity, percent);
  driveRight.setVelocity(velocity, percent);
  driveLeft.spinFor(forward, inches * 49.8637150129, degrees, false); 
  driveRight.spinFor(forward, inches * 49.8637150129, degrees, true);
}
 
// Drive Backward
void driveBackward(double inches, rotationUnits degrees, double velocity, percentUnits pct) {
  driveLeft.setVelocity(velocity, percent);
  driveRight.setVelocity(velocity, percent);
  driveLeft.spinFor(reverse, inches * 49.8637150129, degrees, false); 
  driveRight.spinFor(reverse, inches * 49.8637150129, degrees, true);
}

// Turn right
void turnRight(double inches, rotationUnits degrees, double velocity, percentUnits pct) {
  driveLeft.setVelocity(velocity, percent);
  driveRight.setVelocity(velocity, percent);
  driveLeft.spinFor(forward, inches * 49.8637150129, degrees, false); 
  driveRight.spinFor(reverse, inches * 49.8637150129, degrees, true);
}

// Turn left
void turnLeft(double inches, rotationUnits degrees, double velocity, percentUnits pct) {
  driveLeft.setVelocity(velocity, percent);
  driveRight.setVelocity(velocity, percent);
  driveLeft.spinFor(reverse, inches * 49.8637150129, degrees, false); 
  driveRight.spinFor(forward, inches * 49.8637150129, degrees, true);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  
  // Match Autonomous
  // Set motor velocities
  flywheel.setVelocity(3600, rpm);
  intake.setVelocity(100, percent);

  // Beginning of autonomous task
  driveForward(9, degrees, 10, pct);
  turnLeft(90, degrees, 10, pct);
  driveBackward(2.2, degrees, 20, pct);
  wait(1, sec);
  intake.spinFor(forward, 0.3, turns);
  wait(1, sec);
  driveForward(2, degrees, 10, pct);

  
  // Skills Autonomous (40 + 12 + points from string launcher)
  //setup stuff
  flywheel.setVelocity(3600, rpm);
  intake.setVelocity(100, percent);
  intake.setVelocity(100, percent);

  //beginning of auton task
  intake.spinFor(forward, 2, turns);
  driveForward(15, degrees, 30, pct);
  turnRight(90, degrees, 10, pct);
  driveBackward(5, degrees, 60, pct);
  intake.spinFor(reverse, 2, turns);
  driveForward(15, degrees, 30, pct);
  turnLeft(45, degrees, 10, pct);
  driveForward(40, degrees, 30, pct);
  turnLeft(135, degrees, 10, pct);
  driveBackward(5, degrees, 60, pct);
  intake.spinFor(reverse, 2, turns);
  driveForward(15, degrees, 30, pct);
  turnRight(90, degrees, 10, pct);
  driveBackward(5, degrees, 60, pct);
  intake.spinFor(forward, 2, turns);
  driveForward(15, degrees, 30, pct);
  turnRight(45, degrees, 10, pct);
  driveForward(15, degrees, 30, pct);
  turnLeft(90, degrees, 10, pct);
  stringLauncher.set(1);
  
  /*
  intake.spinFor(forward, 999, turns, false);
  driveForward(41, degrees, 60, pct);
  turnRight(90, degrees, 60, pct);
  intake.stop();
  driveBackward(41, degrees, 60, pct);
  intake.spinFor(forward, 2, turns);

  */
  
  
  /* Comment out all auton PID code for now
  enabledrivePID = true;
  vex::task AutonomousDrive(drivePID);

  resetDriveSensors = true;
  desiredValue = 300;
  desiredTurnValue = 600;

  vex::task::sleep(1000);

  resetDriveSensors = true;
  desiredValue = 300;
  desiredTurnValue = 300; */
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code
  enabledrivePID = false;
  
  stringLauncher.set(0);
  bool launchString = true;
  
  while (1) {
    // Set motor velocities
    flywheel.setVelocity(3600, rpm);
    intake.setVelocity(80, percent);

    // Define button press actions
    // Intake / Roller-Roller
    if (controller1.ButtonL1.pressing()){
      intake.spin(forward);
    }
    else if (controller1.ButtonL2.pressing()){
      intake.spin(reverse);
    }
    else {
      intake.stop();
    }

    // Flywheel
    if (controller1.ButtonR1.pressing()){
      flywheel.spin(forward);
    }
    else if (controller1.ButtonR2.pressing()){
      flywheel.spin(reverse);
    }
    else {
      flywheel.stop();
    }

    // String Launcher Piston
    if (controller1.ButtonA.pressing()) {
      if (launchString) {
        stringLauncher.set(1);
        launchString = false;
        wait(10, msec);
      }
      else {
        stringLauncher.set(0);
        launchString = true;
        wait(10, msec);
      }
    }

    // Define joystick control
    int power = controller1.Axis3.position();
    int turn = controller1.Axis1.position();
    int left = power + turn;
    int right = power - turn;
    int leftVolts = 12.0 * (left / 100.0);
    int rightVolts = 12.0 * (right / 100.0);
    
    driveLeft.spin(forward, leftVolts, volt);
    driveRight.spin(forward, rightVolts, volt);
    wait(10, msec);
  }

}

// Main function will set up the competition functions and callbacks.

int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  Brain.Screen.drawImageFromBuffer(image, 0, 0, sizeof(image));
  
  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}