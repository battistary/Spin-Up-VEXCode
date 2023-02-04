/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Ryan Battista                                             */
/*    Created:      Wed Jun 8 2022                                            */
/*    Description:  9909Y Yottabyte 2022-2023 VEX Spin Up Code                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

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
  driveLeftCenter.setBrake(coast);
  driveLeftFront.setBrake(coast);
  driveRightBack.setBrake(coast);
  driveRightCenter.setBrake(coast);
  driveRightFront.setBrake(coast);

  driveLeft.setTimeout(6, seconds);
  driveRight.setTimeout(6, seconds);

  intake.setBrake(coast);
  flywheel.setBrake(coast);

  stringLauncher.set(0);

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

void driveForward(double inches, double velocity, percentUnits pct) {
  driveLeft.setVelocity(velocity, percent);
  driveRight.setVelocity(velocity, percent);
  driveLeft.spinFor(forward, inches / (3.25 * M_PI / 360 * 0.75), degrees, false);
  driveRight.spinFor(forward, inches / (3.25 * M_PI / 360 * 0.75), degrees, true);
}

void driveBackward(double inches, double velocity, percentUnits pct) {
  driveLeft.setVelocity(velocity, percent);
  driveRight.setVelocity(velocity, percent);
  driveLeft.spinFor(reverse, inches / (3.25 * M_PI / 360 * 0.75), degrees, false);
  driveRight.spinFor(reverse, inches / (3.25 * M_PI / 360 * 0.75), degrees, true);
}

// Turn left
void turnLeft(double turn, double velocity) {
  driveLeft.spin(reverse, velocity, percent);
  driveRight.spin(forward, velocity, percent);
  waitUntil(inertialSensor.rotation(degrees) <= turn);
  driveLeft.stop();
  driveRight.stop();
}

// Turn right
void turnRight(double turn, double velocity) {
  driveLeft.spin(forward, velocity, percent);
  driveRight.spin(reverse, velocity, percent);
  waitUntil(inertialSensor.rotation(degrees) >= turn);
  driveLeft.stop();
  driveRight.stop();
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
  intake.setVelocity(100, pct);
  flywheel.setVelocity(100, pct);
  stringLauncher.set(0);
  
  if (autonToRun == 1) {
    // Left side (AWP) match autonomous (centered width-align, front length-align)
    driveBackward(7, 20, pct);
    intake.spinFor(forward, 240, degrees);
  }
  else if (autonToRun == 2) {
    // Old right side match autono (right width-align, front length-align, turned 90* counterclockwise)
    driveBackward(20, 20, pct);
    turnRight(83, 10);
    driveBackward(9, 20, pct);
    intake.spinFor(forward, 220, degrees);

    // AWP right side match auton (right width-align, front length-align, facin forwards)
    //driveForward(41, 20, pct);
    //turnRight(30, 10);
    //flywheel.spin(forward, 12, volt);
    //intake.spinFor(forward, 90, degrees);
    //wait(1, seconds);
    //intake.spinFor(forward, 90, degrees);
    //wait(1, seconds);
    //intake.spinFor(forward, 90, degrees);
    //wait(1, seconds);
    //turnLeft(-45, 10);
    //driveBackward(42.72, 20, pct);
    //turnRight(0, 10);
    //driveBackward(24, 20, pct);
    //intake.spinFor(forward, 240, degrees);
  }
  else if (autonToRun == 3) {
    /* Skills autonomous (left side, left width-align, front length-align)
    (91 points: 40 (roller) + 12 (bot) + 39 (string launcher) */
    driveBackward(6, 20, pct);
    intake.spinFor(forward, 330, degrees, false);
    wait(1.5, sec);
    driveForward(19, 20, pct);
    turnRight(83, 10);
    driveBackward(25, 20, pct);
    intake.spinFor(forward, 330, degrees, false);
    wait(1.5, sec);
    driveForward(22.25, 20, pct);
    turnLeft(47, 10);

    driveForward(250, 20, pct);
    driveBackward(5, 20, pct);
    turnRight(173, 10);
    driveBackward(23.5, 20, pct); // Decreased 1.5" -- working
    intake.spinFor(forward, 330, degrees, false);
    wait(1.5, sec);
    driveForward(22, 20, pct);
    turnRight(262, 10);
    driveBackward(23, 20, pct); // Increased .5"
    intake.spinFor(forward, 330, degrees, false);
    wait(1.5, sec);
    driveForward(24, 20, pct);
    turnLeft(227, 10);
    driveBackward(15, 20, pct);
    
    //stringLauncher.set(1);
  }
  else {
    // Autonomous to run when no autonomous is selected on brain
    driveBackward(7, 20, pct);
    intake.spinFor(forward, 240, degrees);
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
  
  stringLauncher.set(0);
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
        stringLauncher.set(1);
        launchString = false;
        wait(500, msec);
      }
      else {
        stringLauncher.set(0);
        launchString = true;
        wait(500, msec);
      }
    }

    // One tile drive function test
    //if (controller1.ButtonRight.pressing()) {
    //  driveForward(24, 20, pct);
    //}

    // 90 Degree turn function test
    //if (controller1.ButtonLeft.pressing()) {
    //  turnLeft(90, 20, pct);
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