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
// stringLauncher1       digital_out  A
// stringLauncher2       digital_out  B
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

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
  inertialSensor.calibrate();

  Brain.Screen.drawImageFromFile("brain_logo.png", 0, 0);

  leftEncoder.setPosition(0, degrees);
  rightEncoder.setPosition(0, degrees);
  backEncoder.setPosition(0, degrees);

  driveLeftBack.setBrake(coast);
  driveLeftBack.setTimeout(5, seconds);
  driveLeftCenter.setBrake(coast);
  driveLeftCenter.setTimeout(5, seconds);
  driveLeftFront.setBrake(coast);
  driveLeftFront.setTimeout(5, seconds);

  driveRightBack.setBrake(coast);
  driveRightBack.setTimeout(5, seconds);
  driveRightCenter.setBrake(coast);
  driveRightCenter.setTimeout(5, seconds);
  driveRightFront.setBrake(coast);
  driveRightFront.setTimeout(5, seconds);

  intake.setBrake(coast);
  flywheel.setBrake(coast);

  stringLauncher1.set(0);
  stringLauncher2.set(0);

  driveRightCenter.setTimeout(3, seconds);
  driveRightBack.setTimeout(3, seconds);
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
double desiredValue = 200;
double desiredTurnValue = 0;

double error; // SensorValue - DesiredValue : position
double prevError = 0; // error 20 milliseconds ago
double derivative; // error - prevError (speed)
double totalError = 0; //totalError = totalError + error (integral)

double turnError; //SensorValue - DesiredValue : position
double turnPrevError = 0; // error 20 milliseconds ago
double turnDerivative; // error - prevError (speed)
double turnTotalError = 0; //totalError = totalError + error (integral)

bool resetDriveSensors = false;
bool enabledrivePID = false;

double drivePID() {

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
    double leftEncoderPosition = leftEncoder.position(degrees);
    double rightEncoderPosition = rightEncoder.position(degrees);

    /*******************************************/
    /*          Lateral Movement PID           */
    /*******************************************/
    
    // Get average of the two shaft encoders
    double averagePosition = (leftEncoderPosition + rightEncoderPosition) / 2;
    
    // Potential
    error = (desiredValue * 49.8637150129) - averagePosition;

    // Derivative
    derivative = error - prevError;

    double LateralMotorPower = (error * kP + derivative * kD) / 12.0;

    /*******************************************/
    /*          Turning Movement PD            */
    /*******************************************/

    // Get average of the two motors
    double turnDifference = leftEncoderPosition - rightEncoderPosition;
    
    // Potential
    turnError = ((desiredTurnValue / 360) * 3.256 * 360) - turnDifference;

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
/*                           Auton Driving Functions                         */
/*---------------------------------------------------------------------------*/

// Drive Forward 
void driveForward(double inches, rotationUnits turns, double velocity, percentUnits pct) {
  driveLeft.setVelocity(velocity, percent);
  driveRight.setVelocity(velocity, percent);
  driveLeft.spinFor(forward, inches / (3.25 * M_PI) / (4/3), turns, false);
  driveRight.spinFor(forward, inches / (3.25 * M_PI) / (4/3), turns, true);
}

// Drive Backward
void driveBackward(double inches, rotationUnits turns, double velocity, percentUnits pct) {
  driveLeft.setVelocity(velocity, percent);
  driveRight.setVelocity(velocity, percent);
  driveLeft.spinFor(reverse, inches / (3.25 * M_PI) / (4/3), turns, false);
  driveRight.spinFor(reverse, inches / (3.25 * M_PI) / (4/3), turns, true);
}

// Turn right
//void turnRight(double botDegrees, rotationUnits degrees, double velocity, percentUnits pct) {
//  driveLeft.setVelocity(velocity, percent);
//  driveRight.setVelocity(velocity, percent);
//  driveLeft.spinFor(forward, ((botDegrees * 0.0959444444444444) / 2) / (3.25 * M_PI) / (4/3), degrees, false);
//  driveRight.spinFor(reverse, ((botDegrees * 0.0959444444444444) / 2) / (3.25 * M_PI) / (4/3), degrees, true);
//}
//
//// Turn left
//void turnLeft(double botDegrees, rotationUnits degrees, double velocity, percentUnits pct) {
//  driveLeft.setVelocity(velocity, percent);
//  driveRight.setVelocity(velocity, percent);
//  driveLeft.spinFor(forward, ((botDegrees * 0.0959444444444444) / 2) * (3.25 * M_PI) * (3/4), degrees, false);
//  driveRight.spinFor(reverse, ((botDegrees * 0.0959444444444444) / 2) * (3.25 * M_PI) * (3/4), degrees, true);
//}

// Inertial turn left
void iTurnLeft(double turn, double velocity) {
  driveLeft.spin(reverse, velocity, percent);
  driveRight.spin(forward, velocity, percent);
  waitUntil(inertialSensor.rotation(degrees) <= (turn / 2));
  driveLeft.stop();
  driveRight.stop();
}

// Inertial turn right
void iTurnRight(double turn, double velocity) {
  driveLeft.spin(forward, velocity, percent);
  driveRight.spin(reverse, velocity, percent);
  waitUntil(inertialSensor.rotation(degrees) >= turn);
  driveLeft.stop();
  driveRight.stop();
}

void newDriveForward(double inches, double velocity, percentUnits pct) {
  driveLeft.setVelocity(velocity, percent);
  driveRight.setVelocity(velocity, percent);
  driveLeft.spinFor(forward, inches / (((360 * M_PI) / 360) * (3/4)), degrees, false);
  driveRight.spinFor(forward, inches / (((360 * M_PI) / 360) * (3/4)), degrees, true);
}

void newDriveBacward(double inches, double velocity, percentUnits pct) {
  driveLeft.setVelocity(velocity, percent);
  driveRight.setVelocity(velocity, percent);
  driveLeft.spinFor(reverse, inches / (((360 * M_PI) / 360) * (3/4)), degrees, false);
  driveRight.spinFor(reverse, inches / (((360 * M_PI) / 360) * (3/4)), degrees, true);
}

/*---------------------------------------------------------------------------*/
/*                             Autonomous Selector                           */
/*---------------------------------------------------------------------------*/

int autonToRun = 0;

class Button
{
  public:
    int x, y, width, height;
    std::string text;
    vex::color buttonColor, textColor;
    
    Button(int x, int y, int width, int height, std::string text, vex::color buttonColor, vex::color textColor)
    : x(x), y(y), width(width), height(height), text(text), buttonColor(buttonColor), textColor(textColor){}

    void render()
    {
      Brain.Screen.drawRectangle(x, y, width, height, buttonColor);
      Brain.Screen.printAt(x + 10, y + 10, false, text.c_str());
    }

    bool isClicked()
    {
      if(Brain.Screen.pressing() && Brain.Screen.xPosition() >= x && Brain.Screen.xPosition() <= x + width &&
      Brain.Screen.yPosition() >= y && Brain.Screen.yPosition() <= y + width) return true;
      return false;
    }
};

Button autonButtons[] = {
  Button(10, 10, 150, 50, "Left Match (AWP)", 0xFF0000, 0xFFFFFF),
  Button(170, 10, 150, 50, "Right Match", 0x0000FF, 0xFFFFFF),
  Button(10, 70, 310, 50, "Skills", 0xFF00FF, 0xFFFFFF)
};

vex::color unpressedColors[3] = {0xFF0000, 0x0000FF, 0xFF00FF};

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
  // Initialize devices
  stringLauncher1.set(0);
  stringLauncher2.set(0);
  
  if (autonToRun == 1) {
    // Left side (AWP) match autonomous (centered width-align, front length-align)
    
    driveBackward(6.5, turns, 20, pct);
    intake.spinFor(forward, 200, degrees);
    driveForward(3.25, turns, 20, pct);
    iTurnRight(90, 60);
    driveForward(48, turns, 20, pct);

    iTurnLeft(0, 60);
    driveForward(24, turns, 20, pct);
    intake.spin(forward, 11.8, volt);
    driveForward(24, turns, 20, pct);
    intake.stop();
    iTurnLeft(-45, 60);
    driveForward(4.596194077712559, turns, 20, pct);

    flywheel.spin(forward, 11, volt);
    wait(2, seconds);
    intake.spin(reverse, 11.8, volt);
    wait(5, seconds);
    flywheel.stop();
    intake.stop();
  }
  else if (autonToRun == 2) {
    // Right side match autonomous (right width-align, front length-align, turned 90* counterclockwise)

    driveBackward(34, turns, 20, pct);
    iTurnRight(85, 20);
    driveBackward(21, turns, 20, pct);
    intake.spinFor(forward, 200, degrees);

//    //intake.spin(forward, 11.8, volt);
//    driveForward(20.75, turns, 20, pct);
//    //intake.stop();
//    iTurnLeft(-45, 60);
//    driveForward(33.94112549695428, turns, 20, pct); // little longer
//    iTurnRight(45, 60);
//    driveForward(4.596194077712559, turns, 20, pct);
//
//    flywheel.spin(forward, 11, volt);
//    wait(2, seconds);
//    intake.spin(reverse, 11.8, volt);
//    wait(5, seconds);
//    flywheel.stop();
//    intake.stop();
//    
//    driveBackward(4.596194077712559, turns, 20, pct); 
//    iTurnLeft(-45, 60);
//    driveBackward(67.88225099390856, turns, 20, pct); //little longer
//    iTurnRight(0, 60); //less 
//    driveBackward(3.25, turns, 20, pct);
//    intake.spinFor(forward, 200, degrees);
  }
  else if (autonToRun == 3) {
    // Skills autonomous (left side, left width-align, front length-align) (62 points: 35 + 15 + 12 points from string launcher)
    
    driveBackward(34, turns, 20, pct);
    iTurnRight(85, 20);
    driveBackward(21, turns, 20, pct);
    intake.spinFor(forward, 200, degrees);

    driveForward(10, turns, 20, pct);
    iTurnLeft(0, 20);
    driveBackward(21, turns, 20, pct);
    intake.spinFor(forward, 200, degrees);

    driveForward(15, turns, 20, pct);
    iTurnLeft(90, 20);
    driveForward(10, turns, 20, pct);
    iTurnRight(225, 20);

    stringLauncher1.set(1);
    stringLauncher2.set(1);

    //driveBackward(6.5, turns, 20, pct);
    //intake.spinFor(forward, 200, degrees);
//
    ////intake.spin(forward, 11.8, volt);
    //driveForward(24, turns, 20, pct);
    ////intake.stop();
    //iTurnRight(90, 20);
    //driveBackward(24, turns, 20, pct);
    //intake.spinFor(forward, 200, degrees);
//
    //driveForward(60, turns, 20, pct);
    //iTurnLeft(0, 60);
    //driveForward(66.5, turns, 20, pct);
    //iTurnLeft(-90, 60);
    //driveBackward(54.5, turns, 20, pct);
    //intake.spinFor(forward, 200, degrees);
//
//
//
    //driveForward(4.596194077712559, turns, 20, pct);
//
    //flywheel.spin(forward, 11, volt);
    //wait(2, seconds);
    //intake.spin(reverse, 11.8, volt);
    //wait(2, seconds);
    //flywheel.stop();
    //intake.stop();
    //
    //driveBackward(4.596194077712559, turns, 20, pct);
    //iTurnRight(0, 60);
    //driveBackward(51.25, turns, 20, pct);
    //stringLauncher1.set(1);
    //stringLauncher2.set(1);
    //
    ///* Comment out all auton PID code for now
    //enabledrivePID = true;
    //vex::task AutonomousDrive(drivePID);
//
    //resetDriveSensors = true;
    //desiredValue = 300;
    //desiredTurnValue = 600;
//
    //vex::task::sleep(1000);
//
    //resetDriveSensors = true;
    //desiredValue = 300;
    //desiredTurnValue = 300; */
  }
  else {
    // Autonomous to run when no autonomous is selected on brain
    //driveBackward(4, turns, 20, pct);
    //intake.spinFor(forward, 200, degrees);

    newDriveForward(24, 20, pct);
  }
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
  
  stringLauncher1.set(0);
  stringLauncher2.set(0);
  bool launchString = true;
  
  while (1) {
    // Define button press actions
    // Intake / Roller-Roller
    if (controller1.ButtonL1.pressing()){
      intake.spin(reverse, 11.8, volt);
    }
    else if (controller1.ButtonL2.pressing()){
      intake.spin(forward, 11.8, volt);
    }
    else {
      intake.stop();
    }

    // Flywheel
    if (controller1.ButtonR1.pressing()){
      flywheel.spin(forward, 11, volt);
    }
    else {
      flywheel.stop();
    }

    // String Launcher Piston
    if (controller2.ButtonA.pressing()) {
      if (launchString) {
        stringLauncher1.set(1);
        stringLauncher2.set(1);
        launchString = false;
        wait(500, msec);
      }
      else {
        stringLauncher1.set(0);
        stringLauncher2.set(0);
        launchString = true;
        wait(500, msec);
      }
    }

    // One tile drive function test
    //if (controller1.ButtonRight.pressing()) {
    //  driveForward(24, turns, 20, pct);
    //}

    // 90 Degree turn function test
    //if (controller1.ButtonLeft.pressing()) {
    //  turnLeft(90, turns, 20, pct);
    //}

    // Define joystick control
    double power = controller1.Axis3.position();
    double turn = controller1.Axis1.position();
    double left = power + turn;
    double right = power - turn;
    double leftVolts = 12.0 * (left / 100.0);
    double rightVolts = 12.0 * (right / 100.0);

    driveLeft.spin(forward, leftVolts, voltageUnits::volt);
    driveRight.spin(forward, rightVolts, voltageUnits::volt);

    wait(10, msec);
  }

}

// Main function will set up the competition functions and callbacks.

int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  
  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {

    // Render autonomous selector on brain
    if (!Competition.isEnabled()) {
      for(int i = 0; i < 3; i++) {
        autonButtons[i].render();
        if(autonButtons[i].isClicked()) {
          autonButtons[autonToRun].buttonColor = unpressedColors[autonToRun];
          autonButtons[i].buttonColor = 0x00FF00;
          autonToRun = i + 1;
        }
      }
    }
    else {
      Brain.Screen.drawImageFromFile("brain_logo.png", 0, 0);
    }
    
    wait(100, msec);
  }
}