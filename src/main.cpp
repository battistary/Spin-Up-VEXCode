/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Ryan Battista and Shrikar Seshadri                        */
/*    Created:      Wed Jun 8 2022                                            */
/*    Description:  X-Drive with Odometrey Program                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// frontLeft            motor         11              
// backLeft             motor         12              
// frontRight           motor         1               
// backRight            motor         2               
// Controller1          controller                    
// leftEncoder          encoder       A, B            
// rightEncoder         encoder       C, D            
// backEncoder          encoder       E, F            
// intake               motor_group   3, 4            
// flywheel             motor         15              
// rollerRoller         motor         5               
// magazinePiston       digital_out   H               
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
}

/*---------------------------------------------------------------------------*/
/*                          Constants and Variables                          */
/*---------------------------------------------------------------------------*/
int prevleftEncoderPosition;
int prevrightEncoderPosition;
int prevbackEncoderPosition;

double totalDistanceleftEncoder = 0; // delta L sub r
double totalDistancerightEncoder = 0; // delta R sub r
double totalDistancebackEncoder = 0; // delta R sub r

double relOrientation; // Orientation of the robot based on the distances travelled by the tracking wheels
double prevOrientation = 0; // Orientation 10 milliseconds ago
double deltaOrientation; // absOrientation - prevOrientation: change in orientation in 10 milliseconds
double avgOrientation; // average orientation of the robot at any time (absolute)

double TsubL = 7.250; // distance between center of robot and left tracking wheel
double TsubR = 7.250; // distance between center of robot and left tracking wheel
double TsubS = 7.250; // distance between center of robot and back tracking wheel

double XPosition; // robot's relative X-coordinate (measured by the distance travelled by the back tracking wheel)
double YPosition; // robot's relative Y-coordinate (measured by the distance travelled the left and right tracking wheels)
double prevXPosition = 0; // robot's X coordinate 10 milliseconds ago
double prevYPosition = 0; // robot's Y coordinate 10 milliseconds ago 
double absXPosition; // absolute X position of robot
double absYPosition; // absolute Y position of robot

double constant = 49.8637150129; // conversion from degrees travelled by the shaft encoder to inches travelled (depends on size of tracking wheel)

/*---------------------------------------------------------------------------*/
/*                        Tracking Odometry Function                         */
/*---------------------------------------------------------------------------*/
int TrackingOdometry(){
  int leftEncoderPosition = leftEncoder.position(degrees);
  int rightEncoderPosition = rightEncoder.position(degrees);
  int backEncoderPosition = backEncoder.position(degrees);

  double distleftEncoder = (leftEncoderPosition - prevleftEncoderPosition)/constant; //delta L
  double distrightEncoder = (rightEncoderPosition - prevrightEncoderPosition)/constant; //delta R
  double distbackEncoder = (backEncoderPosition - prevbackEncoderPosition)/constant; //delta S

  totalDistanceleftEncoder += distleftEncoder; // delta L sub r
  totalDistancerightEncoder += distrightEncoder; // delta R sub r
  totalDistancebackEncoder += distbackEncoder; //delta S sub r

  relOrientation = prevOrientation + ((distleftEncoder - distrightEncoder)/(TsubL + TsubR))*(180/M_PI);
  double deltaOrientation = relOrientation - prevOrientation;

  if(deltaOrientation == 0) {
    // calculates local offset given that the robot hasn't turned --> relative positions, not global
    XPosition = distbackEncoder;
    YPosition = distrightEncoder;
  }
  else {
    // calculates the relative (not global) x-coordinate position of the robot
    XPosition = (2*sin((relOrientation*M_PI/180)/2)*((distbackEncoder/deltaOrientation) + TsubS));

    // calculates the relative y-coordinate (not global) position of the robot
    YPosition = (2*sin((relOrientation*M_PI/180)/2)*((distrightEncoder/deltaOrientation) + TsubR));
  }

  // calculates the average orientation of the robot 
  avgOrientation = prevOrientation + (deltaOrientation/2);

  /* Below steps calculate the global position of the robot (absXPosition, absYPosition) by rotating the                          *
   * polar coordinate equivelants of XPosition and YPosition by -avgOrientation and then converting back to cartesian coordinates */

  // calculates the r value in the polar coordinate system
  double rValue = (XPosition)*(XPosition) + (YPosition)*(YPosition);

  // calculates theta (IN RADIANS) in the polar coordinate system
  double theta = atan(YPosition/XPosition);
  
  // changes the angle theta so that the local offset becomes a global position
  theta = theta - (avgOrientation*M_PI/180);

  // recalculates the relative cartesian coordinates of the robot with the adjusted angle
  XPosition = rValue*cos(theta);
  YPosition = rValue*sin(theta);

  // calculates the absolute X and Y Positions
  absXPosition = prevXPosition + XPosition;
  absYPosition = prevYPosition + YPosition;

  // Move odom values to prev variables to store for next call
  leftEncoderPosition = prevleftEncoderPosition;
  rightEncoderPosition = prevrightEncoderPosition;
  backEncoderPosition = prevbackEncoderPosition;

  relOrientation = prevOrientation;

  XPosition = prevXPosition;
  YPosition = prevYPosition;

  vex::task::sleep(10);
  return 0;
}

/*---------------------------------------------------------------------------*/
/*                        Turning Odometry Function                          */
/*---------------------------------------------------------------------------*/
double turnkP = 0.0;
double turnkI = 0.0;
double turnkD = 0.0;

double desiredOrientation;
double turnError;
double turnDerivative;
double turnPrevError;
double turnTotalError;

int TurningOdom(double deg, rotationUnits degrees){
  vex::task TurningOdom(TrackingOdometry);
  desiredOrientation = deg;

  // PD Loop (No Integral yet due to certain problems that arise from using it)

  // Potential
  turnError = desiredOrientation - relOrientation; //not sure if I should use relOrientation or avgOrientation here (??) 

  // Integral
  // turnTotalError += turnError;

  // Derivative
  turnDerivative = turnError - turnPrevError;

  double TurnMotorPower = (turnError*turnkP + turnDerivative*turnkD)/12.0;

  if (desiredOrientation > relOrientation){
    //sets motor velocities based on the PD controller - does this work simoultaneously with the spinning code? Need to check
    frontLeft.spin(reverse, TurnMotorPower, voltageUnits::volt);
    frontRight.spin(forward, TurnMotorPower, voltageUnits::volt);
    backRight.spin(forward, TurnMotorPower, voltageUnits::volt);
    backLeft.spin(reverse, TurnMotorPower, voltageUnits::volt); 

    //spins the motors for the specified number of degrees
    frontLeft.spinFor(reverse, (deg/360)*3.264*360, degrees, false); 
    frontRight.spinFor(forward, (deg/360)*3.264*360, degrees, false);
    backRight.spinFor(forward, (deg/360)*3.264*360, degrees, false); 
    backLeft.spinFor(reverse, (deg/360)*3.264*360, degrees, false);
  }

  if (desiredOrientation < relOrientation){
    //sets motor velocities based on the PD controller - does this work simoultaneously with the spinning code? Need to check
    frontLeft.spin(forward, TurnMotorPower, voltageUnits::volt);
    frontRight.spin(reverse, TurnMotorPower, voltageUnits::volt);
    backRight.spin(reverse, TurnMotorPower, voltageUnits::volt);
    backLeft.spin(forward, TurnMotorPower, voltageUnits::volt); 

    //spins the motors for the specified number of degrees
    frontLeft.spinFor(forward, (deg/360)*3.264*360, degrees, false); 
    frontRight.spinFor(reverse, (deg/360)*3.264*360, degrees, false);
    backRight.spinFor(reverse, (deg/360)*3.264*360, degrees, false); 
    backLeft.spinFor(forward, (deg/360)*3.264*360, degrees, false);
  }

  turnPrevError = turnError;
  vex::task::sleep(10);
  return 0;
}

/*---------------------------------------------------------------------------*/
/*                           MovetoPoint Function                            */
/*---------------------------------------------------------------------------*/
int MovetoPoint(double XDestination, double YDestination, rotationUnits degrees){
  vex::task MovingOdom(TrackingOdometry);

  double XDistance = XDestination - absXPosition;
  double YDistance = YDestination - absYPosition;

  double hypotenuseDistance = sqrt((XDistance)*(XDistance) + (YDistance)*(YDistance)); // calculates distance that needs to be travelled

  double angle = atan(YDistance/XDistance)*(180/M_PI); //calculates the angle (in degrees) that the robot needs to go at to reach the point

  //calculates what angle to take depending on what the robot's orientation is and where it needs to go 
  if (relOrientation > angle){
    //turns the robot to the right given that the orientation of the robot is greater than the angle that the robot needs to travel at 
    frontLeft.spinFor(forward, ((relOrientation - angle)/360)*3.256*360, degrees, false); 
    frontRight.spinFor(reverse, ((relOrientation - angle)/360)*3.256*360, degrees, false);
    backRight.spinFor(reverse, ((relOrientation - angle)/360)*3.256*360, degrees, false); 
    backLeft.spinFor(forward, ((relOrientation - angle)/360)*3.256*360, degrees, false);

    //moves the robot the specified hypotenuse distance
    frontLeft.spinFor(forward, hypotenuseDistance*49.8637150129, degrees, false); 
    frontRight.spinFor(forward, hypotenuseDistance*49.8637150129, degrees, false); 
    backLeft.spinFor(forward, hypotenuseDistance*49.8637150129, degrees, false); 
    backRight.spinFor(forward, hypotenuseDistance*49.8637150129, degrees, false);
  }
  else if (relOrientation < angle){
    //turns the robot to the left given that the orientation of the robot is less than the angle that the robot needs to travel at
    frontLeft.spinFor(reverse, ((angle - relOrientation)/360)*3.264*360, degrees, false); 
    frontRight.spinFor(forward, ((angle - relOrientation)/360)*3.264*360, degrees, false);
    backRight.spinFor(forward, ((angle - relOrientation)/360)*3.264*360, degrees, false); 
    backLeft.spinFor(reverse, ((angle - relOrientation)/360)*3.264*360, degrees, false);

    //moves the robot the specified hypotenuse distance
    frontLeft.spinFor(forward, hypotenuseDistance*49.8637150129, degrees, false); 
    frontRight.spinFor(forward, hypotenuseDistance*49.8637150129, degrees, false); 
    backLeft.spinFor(forward, hypotenuseDistance*49.8637150129, degrees, false); 
    backRight.spinFor(forward, hypotenuseDistance*49.8637150129, degrees, false);
  }

  vex::task::sleep(10);
  return 0;
}

/*---------------------------------------------------------------------------*/
/*                         Baseline PID - Don't Use                          */
/*---------------------------------------------------------------------------*/
// Settings - TUNE TO OUR ROBOT
double kP = 0.0;
double kI = 0.0;
double kD = 0.0;
// double turnkP = 0.0;
// double turnkI = 0.0;
// double turnkD = 0.0;

// Autonomous settings
int desiredValue = 200;
int desiredTurnValue = 0;

int error; // SensorValue - DesiredValue : position
int prevError = 0; // error 20 milliseconds ago
int derivative; // error - prevError (speed)
int totalError = 0; //totalError = totalError + error (integral)

// int turnError; //SensorValue - DesiredValue : position
// int turnPrevError = 0; // error 20 milliseconds ago
// int turnDerivative; // error - prevError (speed)
// int turnTotalError = 0; //totalError = totalError + error (integral)

bool resetDriveSensors = false;
bool enabledrivePID = true;

int drivePID() { // not in use - trying to integrate code here into odometry code

  while(enabledrivePID){

    if (resetDriveSensors){
      resetDriveSensors = false;
      frontLeft.setPosition(0, degrees);
      frontRight.setPosition(0, degrees);
      backLeft.setPosition(0, degrees);
      backRight.setPosition(0, degrees);
      leftEncoder.setPosition(0, degrees);
      rightEncoder.setPosition(0, degrees);
    }

    //Get the position of the motors
    int frontLeftMotorPosition = frontLeft.position(degrees);
    int frontRightMotorPosition = frontRight.position(degrees);
    int backLeftMotorPosition = backLeft.position(degrees);
    int backRightMotorPosition = backRight.position(degrees);

    int leftEncoderPosition = leftEncoder.position(degrees);
    int rightEncoderPosition = rightEncoder.position(degrees);

    /*******************************************/
    /*          Lateral Movement PID           */
    /*******************************************/
    
    // Get average of the two motors
    int averagePosition = (leftEncoderPosition + rightEncoderPosition)/2;
    
    // Potential
    error = (desiredValue*49.8637150129) - averagePosition;

    // Derivative
    derivative = error - prevError;
    
    // Integral
    totalError += error;

    double LateralMotorPower = (error*kP + derivative*kD + totalError*kI)/12.0;

    /*******************************************/
    /*          Turning Movement PID           */
    /*******************************************/

    // Get average of the two motors
    int turnDifference = leftEncoderPosition - rightEncoderPosition;
    
    // Potential
    turnError = ((desiredTurnValue/360)*3.256*360) - turnDifference;

    // Derivative
    turnDerivative = turnError - turnPrevError;
    
    // Integral
    turnTotalError += turnError;

    /////////////////////////////////////////////

    double TurnMotorPower = (turnError*turnkP + turnDerivative*turnkD + turnTotalError*turnkI)/12.0;
    
    frontLeft.spin(forward, LateralMotorPower + TurnMotorPower, voltageUnits::volt);
    frontRight.spin(forward, LateralMotorPower - TurnMotorPower, voltageUnits::volt);
    backLeft.spin(forward, LateralMotorPower + TurnMotorPower, voltageUnits::volt);
    backRight.spin(forward, LateralMotorPower - TurnMotorPower, voltageUnits::volt);

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
void driveForward(double inches, rotationUnits degrees, double velocity, percentUnits pct){
  frontLeft.setVelocity(velocity, percent);
  frontRight.setVelocity(velocity, percent);
  backLeft.setVelocity(velocity, percent);
  backRight.setVelocity(velocity, percent);
  frontLeft.spinFor(forward, inches*49.8637150129, degrees, false); 
  frontRight.spinFor(forward, inches*49.8637150129, degrees, false); 
  backLeft.spinFor(forward, inches*49.8637150129, degrees, false); 
  backRight.spinFor(forward, inches*49.8637150129, degrees, true);
}
 
// Drive Backward
void driveBackward(double inches, rotationUnits degrees, double velocity, percentUnits pct){
  frontLeft.setVelocity(velocity, percent);
  frontRight.setVelocity(velocity, percent);
  backLeft.setVelocity(velocity, percent);
  backRight.setVelocity(velocity, percent);
  frontLeft.spinFor(reverse, inches*49.8637150129, degrees, false); 
  frontRight.spinFor(reverse, inches*49.8637150129, degrees, false); 
  backLeft.spinFor(reverse, inches*49.8637150129, degrees, false); 
  backRight.spinFor(reverse, inches*49.8637150129, degrees, true); 
}

// Drive Left
void driveLeft(double inches, rotationUnits degrees, double velocity, percentUnits pct){
  frontLeft.setVelocity(velocity, percent);
  frontRight.setVelocity(velocity, percent);
  backLeft.setVelocity(velocity, percent);
  backRight.setVelocity(velocity, percent);
  frontLeft.spinFor(reverse, inches*49.8637150129, degrees, false); 
  frontRight.spinFor(forward, inches*49.8637150129, degrees, false);
  backRight.spinFor(reverse, inches*49.8637150129, degrees, false); 
  backLeft.spinFor(forward, inches*49.8637150129, degrees, true);
}

// Drive Right
void driveRight(double inches, rotationUnits degrees, double velocity, percentUnits pct){
  frontLeft.setVelocity(velocity, percent);
  frontRight.setVelocity(velocity, percent);
  backLeft.setVelocity(velocity, percent);
  backRight.setVelocity(velocity, percent);
  frontLeft.spinFor(forward, inches*49.8637150129, degrees, false); 
  frontRight.spinFor(reverse, inches*49.8637150129, degrees, false);
  backRight.spinFor(forward, inches*49.8637150129, degrees, false); 
  backLeft.spinFor(reverse, inches*49.8637150129, degrees, true);
}

// Turn right
void turnRight(double deg, rotationUnits degrees, double velocity, percentUnits pct){
  frontLeft.setVelocity(velocity, percent);
  frontRight.setVelocity(velocity, percent);
  backLeft.setVelocity(velocity, percent);
  backRight.setVelocity(velocity, percent);
  frontLeft.spinFor(forward, (deg/360)*3.256*360, degrees, false); 
  frontRight.spinFor(reverse, (deg/360)*3.256*360, degrees, false);
  backRight.spinFor(reverse, (deg/360)*3.256*360, degrees, false); 
  backLeft.spinFor(forward, (deg/360)*3.256*360, degrees, true);
}

// Turn left
void turnLeft(double deg, rotationUnits degrees, double velocity, percentUnits pct){
  frontLeft.setVelocity(velocity, percent);
  frontRight.setVelocity(velocity, percent);
  backLeft.setVelocity(velocity, percent);
  backRight.setVelocity(velocity, percent);
  frontLeft.spinFor(reverse, (deg/360)*3.264*360, degrees, false);
  frontRight.spinFor(forward, (deg/360)*3.264*360, degrees, false);
  backRight.spinFor(forward, (deg/360)*3.264*360, degrees, false);
  backLeft.spinFor(reverse, (deg/360)*3.264*360, degrees, true);
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

void autonomous(void){
  // Current janky auton
  
  //driveBackward(15, degrees, 30, pct);
  //turnRight(90, degrees, 10, pct);
  //driveForward(5, degrees, 60, pct);
  flywheel.setVelocity(3600, rpm);
  intake.setVelocity(100, percent);
  rollerRoller.setVelocity(100, percent);
  rollerRoller.spinFor(forward, 2, turns);
  intake.spinFor(forward, 999, turns, false);
  driveForward(41, degrees, 60, pct);
  turnRight(90, degrees, 60, pct);
  intake.stop();
  driveBackward(41, degrees, 60, pct);
  rollerRoller.spinFor(forward, 2, turns);

  
  
  
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
  
  while (1) {
    // Set motor velocities
    flywheel.setVelocity(3600, rpm);
    intake.setVelocity(100, percent);
    rollerRoller.setVelocity(100, percent);

    // Define button press actions
    // Intake
    if (Controller1.ButtonL1.pressing()){
      intake.setVelocity(80, percent);
      intake.spin(forward);
    }
    else if (Controller1.ButtonL2.pressing()){
      intake.setVelocity(80,percent);
      intake.spin(reverse);
    }
    else {
      intake.stop();
    }

    // flywheel
    if (Controller1.ButtonR1.pressing()){
      flywheel.spin(forward);
    }
    else if (Controller1.ButtonR2.pressing()){
      flywheel.spin(reverse);
    }
    else {
      flywheel.stop();
    }

    // Roller-Roller
    if (Controller1.ButtonUp.pressing()){
      rollerRoller.spin(forward);
    }
    else if (Controller1.ButtonDown.pressing()){
      rollerRoller.spin(reverse);
    }
    else {
      rollerRoller.stop();
    }

    // Magazine Piston
    magazinePiston.set(false);
    if (Controller1.ButtonA.pressing()) {
      magazinePiston.set(true);
      wait(1, seconds);
      magazinePiston.set(false);
    }

    // Define joystick control
    frontLeft.spin(vex::directionType::fwd, (Controller1.Axis3.value() + Controller1.Axis1.value() + Controller1.Axis4.value()), vex::velocityUnits::pct);
    frontRight.spin(vex::directionType::fwd, (Controller1.Axis3.value() - Controller1.Axis1.value() - Controller1.Axis4.value()), vex::velocityUnits::pct);
    backLeft.spin(vex::directionType::fwd, (Controller1.Axis3.value() + Controller1.Axis1.value() - Controller1.Axis4.value()), vex::velocityUnits::pct);
    backRight.spin(vex::directionType::fwd, (Controller1.Axis3.value() - Controller1.Axis1.value() + Controller1.Axis4.value()), vex::velocityUnits::pct);

    wait(20, msec);
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
