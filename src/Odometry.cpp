#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

#include "robot-config.h"

using namespace vex;
// A global instance of competition
competition Competition;

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

  double XBeginning = absXPosition;
  double YBeginning = absYPosition;


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
/*                        Set Points Code                                    */
/*---------------------------------------------------------------------------*/

//creates the set points that will be used as reference for position correction 
double setPoints(double XDestination, double YDestination, rotationUnits degrees) {

  vex::task setPoints(TrackingOdometry);

  double XDistance = XDestination - absXPosition;
  double YDistance = YDestination - absYPosition;
  double XBeginning = absXPosition;
  double YBeginning = absYPosition;
  double hypotenuseDistance = sqrt((XDistance)*(XDistance) + (YDistance)*(YDistance)); // calculates distance that needs to be travelled

  if (hypotenuseDistance < 15){
  //3 set points code
  double setPoint1X = XBeginning + (1/3) * XDistance;
  double setPoint1Y = YBeginning + (1/3) * YDistance;
  double setPoint2X = XBeginning + (2/3) * XDistance;
  double setPoint2Y = YBeginning + (2/3) * YDistance;
  double setPoint3X = XBeginning + XDistance;
  double setPoint3Y = YBeginning + YDistance;

  }

  else {
  
  //sets 5 set points 
  //fill in code here 

  }

 return 0;
};

/*---------------------------------------------------------------------------*/
/*                        Position Correction Code                           */
/*---------------------------------------------------------------------------*/

/*

1. Get the robot's current X and Y positions and heading 

2. See if the robot is currently on the correct path - have to check the heading of the robot and the actual coordinate positions

2a. Compare the current heading of the robot from the Tracking Odometry function with the value of the "angle" variable. 
It will either be equal (don't have to do anything) or greater than or less than (have to correct the position in these cases) 

2b. Get the current X and Y coordinates of the robot from the Tracking Odometry function. Get the hypotenuse distances
(h1 and h2) and check if h1 + h2 = the actual hypotenuse distance between the set points 

3. If the heading and the coordinates are off:
    a. Calculate hypotenuse distance (h) and angle (theta) to the second closest set point
    b. hypotenuse distance = chord length; have to create an arc that gets the robot back onto the path and the proper angle

4. If only the heading is off:

a. Pause, then correct heading by changing angle (add angle - current heading to the current heading), 
  then continue on with program

5. If only the coordinates are off:
    a. Calculate hypotenuse distance (h) and angle (theta) to the second closest set point
    b. hypotenuse distance = chord length; have to create an arc that gets the robot back onto the path and the proper angle

6. If nothing is off:
  a. Do nothing? Not sure

*/