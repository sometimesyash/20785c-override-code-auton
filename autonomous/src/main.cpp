 // ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Motor3               motor         3               
// Motor4               motor         4               
// Inertial20           inertial      20              
// intakeMotor1         motor         5               
// intakeMotor2         motor         6               
// roller1              motor         7               
// roller2              motor         8               
// Motor1               motor         1               
// Motor2               motor         2               
// ---- END VEXCODE CONFIGURED DEVICES ----
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\reply                                            */
/*    Created:      Mon Oct 26 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

//Private GitHub changes and version control avaliabe
//Code using the MARK system as coined by Swift
//Motor 1 and 2 are the RHS
//Motor 3 and 4 are the LHS

//Library Declaration

#include "vex.h"
#include <cstdio>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <cmath>

using namespace vex;

//Variable Declaration

bool leftSideSpin;
bool rightSideSpin;
int desiredValue = 0; //Set to 0 as a default
std::ofstream ofs;

// MARK: Adding a library for ease-of-use

/*

A library drastically reduces the numbers of lines of code by creating a set of 'Void' function -
functions without a return value, that would be made to reduce repeating certain instructions.

*/

void driveForward(int rotations, int timeT){ //gets the motors to drive forward at x speed for y time

  Motor1.spin(forward, rotations, rpm);
  Motor2.spin(forward, rotations, rpm);
  Motor3.spin(forward, rotations, rpm); /*No need to make it minus since the motors will be flipped in setup*/
  Motor4.spin(forward, rotations, rpm);
  
  wait(timeT, msec);

  desiredValue = (timeT/60000) * rotations; //Figuring out the desired value for each movement
  

  Motor1.stop();
  Motor2.stop();
  Motor3.stop();
  Motor4.stop();


}

void driveForwardPID(int rotationsError){ //Small function and corrects the driving actively

  Motor1.spinFor(forward, rotationsError, turns);
  Motor1.spinFor(forward, rotationsError, turns);
  Motor1.spinFor(forward, rotationsError, turns);
  Motor1.spinFor(forward, rotationsError, turns);



}


void driveBackward(int rotations, int timeT){ //gets the motors to drive backwards at x speed for y time

  Motor1.spin(reverse, rotations, rpm);
  Motor2.spin(reverse, rotations, rpm);
  Motor3.spin(forward, rotations, rpm);
  Motor4.spin(forward, rotations, rpm);

  wait(timeT, msec);

  desiredValue = (timeT/60000) * rotations; 

  Motor1.stop();
  Motor2.stop();
  Motor3.stop();
  Motor4.stop();
  
}

void turnRight(int degreesIn, int speed) { //using the inertial sensor to turn 

  Inertial20.resetRotation();

  Motor1.spin(reverse, speed, rpm);
  Motor2.spin(reverse, speed, rpm);
  Motor3.spin(forward, speed, rpm);
  Motor4.spin(forward, speed, rpm);

  waitUntil((Inertial20.rotation(degrees) >= degreesIn)); //Wait until the degrees needed has been reached

  Motor1.stop();
  Motor2.stop();
  Motor3.stop();
  Motor4.stop();

}

void turnLeft(int degreesOut, int speed) { //using the inertial sensor to turn 

  Inertial20.resetRotation();

  Motor1.spin(forward, speed, rpm);
  Motor2.spin(forward, speed, rpm);
  Motor3.spin(reverse, speed, rpm);
  Motor4.spin(reverse, speed, rpm);

  waitUntil((Inertial20.rotation(degrees) >= degreesOut)); //Wait until the degrees needed has been reached

  Motor1.stop();
  Motor2.stop();
  Motor3.stop();
  Motor4.stop();

}

void intake(int speed){ //for the intake

  intakeMotor1.spin(forward, speed, rpm);
  intakeMotor2.spin(forward, speed, rpm);    

}

void rollers(int speed){

  roller1.spin(forward, speed, rpm);
  roller2.spin(forward, speed, rpm);


}


//MARK: The Analysis system

/*

This helps us calculate certain data that can be used to determine the efficency of the motors, the brain etc,
in order to determine whether our design and program is best optimised for the equipment we have. 

*/

bool areSpinning(){ //Checks if the system is spinning or not

  bool isWorking;

  if(Motor1.isSpinning() && Motor2.isSpinning()){

    if(Motor3.isSpinning() && Motor4.isSpinning()){

      rightSideSpin = true;

    }

    leftSideSpin = true;
    isWorking = true;

  } else if(Motor3.isSpinning() && Motor4.isSpinning()){

    rightSideSpin = true;
    isWorking = true;

  } else {
    isWorking = false;
  }


  //helps turn the boolean into a on/off like system that can help determine the state of motors at any given time

  return isWorking; 
  
}


class movement {
  public:

    std::stringstream battery;
    std::stringstream batteryChange;
    std::stringstream timeTaken;
    std::stringstream motor1Torque;
    std::stringstream motor2Torque;
    std::stringstream motor3Torque;
    std::stringstream motor4Torque;
    std::stringstream temperatureChange;

  /*A movement is the method that the system will store the data in*/

};

int calculateAverage(){
  int total = Motor1.temperature(percent) + Motor2.temperature(percent) + Motor3.temperature(percent) + Motor4.temperature(percent);
  int average = total / 4;

  return average;
}

int analyseActions(){

  //Calculate the data

  int battery = Brain.Battery.voltage();
  int batteryChange = 0;
  int timeTaken = 0;
  int motor1Torque = 0;
  int motor2Torque = 0;
  int motor3Torque = 0;
  int motor4Torque = 0;
  int startingTemperature = calculateAverage();
  int temperatureChange = 0;

  if(areSpinning()){

    if(rightSideSpin && leftSideSpin !=true){
      Brain.Timer.reset();
      while(Motor3.isDone() != true){
        motor3Torque = Motor3.torque();
        motor4Torque = Motor4.torque();
      }
      batteryChange = battery - Brain.Battery.current();
      timeTaken = Brain.Timer.value();
      int32_t finalTemp = calculateAverage();
      temperatureChange = startingTemperature - finalTemp;
      

    } else if(leftSideSpin &&  rightSideSpin !=true){
      Brain.Timer.reset();
      while(Motor2.isDone() != true){
        motor1Torque = Motor1.torque();
        motor2Torque = Motor2.torque();
      }
      batteryChange = battery - Brain.Battery.current();
      timeTaken = Brain.Timer.value();
      int32_t finalTemp = calculateAverage();
      temperatureChange = startingTemperature - finalTemp;
      

    } else if(leftSideSpin && rightSideSpin){
      Brain.Timer.reset();
      while(Motor2.isDone() != true){
        motor1Torque = Motor1.torque();
        motor2Torque = Motor2.torque();
      }

      batteryChange = battery - Brain.Battery.current();
      timeTaken = Brain.Timer.value();
      int32_t finalTemp = calculateAverage();
      temperatureChange = startingTemperature - finalTemp;

    }

    movement newMovement;
    newMovement.battery << battery;
    newMovement.batteryChange << batteryChange;
    newMovement.timeTaken << timeTaken; 
    newMovement.motor1Torque << motor1Torque;
    newMovement.motor2Torque << motor2Torque;
    newMovement.motor3Torque << motor3Torque;
    newMovement.motor4Torque << motor4Torque;
    newMovement.temperatureChange << temperatureChange;

    

    if(Brain.SDcard.isInserted()){

      std::string battery1;
      std::string batteryChange1;
      std::string timeTaken1;
      std::string motor1Torque1;
      std::string motor2Torque1;
      std::string motor3Torque1;
      std::string motor4Torque1;
      std::string temperatureChange1;



      ofs.open("analysis.txt", std::ofstream::out); //Making the filename

      //Adding the stringstream into a string

      newMovement.battery >> battery1;
      newMovement.batteryChange >> batteryChange1;
      newMovement.timeTaken >> timeTaken1;
      newMovement.motor1Torque >> motor1Torque1;
      newMovement.motor2Torque >> motor2Torque1;
      newMovement.motor3Torque >> motor3Torque1;
      newMovement.motor4Torque >> motor4Torque1;
      newMovement.temperatureChange >> temperatureChange1;

      ofs << battery1;
      ofs << batteryChange1;
      ofs << timeTaken1;
      ofs << motor1Torque1;
      ofs << motor2Torque1;
      ofs << motor3Torque1;
      ofs << motor4Torque1;
      ofs << temperatureChange1;
      ofs << "\n";

      ofs.close();
      

      Brain.Screen.printAt(10, 40, "done");

    } else {

      Brain.Screen.printAt(10, 40, "No SD Card");

    }

    //Upload the data onto the sd card.

  }


  return 0;
}






int analyse(){

  //Initialising the task3

  while(true){

    analyseActions();

  }
  
  return 0;
}



//MARK: PID

//The use of PID is integral to running code that is actively
//PID Variable Declaration


int error;
int prevError = 0;
int derivative;
int totalError = 0;

//Turn

int tError;
int tPrevError = 0;
int tDerivative;
int tTotalError = 0;
int tDesiredValue;


bool enableDriverPID = true;

int autonPID(){
  
  while(enableDriverPID){

    int leftMotorPosition = (Motor3.position(turns) + Motor4.position(turns))/2;
    int rightMotorPosition = (Motor1.position(turns) + Motor2.position(turns))/2;
    int averagePos = (leftMotorPosition + rightMotorPosition)/2;

    error = averagePos - desiredValue; //Gets the Derivative Error
    
    derivative = error - prevError;

    double lateralMotorPower = error + derivative; //Gets the entire need to change derviative 

    //Turn PID
    int turnDifference = leftMotorPosition - rightMotorPosition; 

    tError = turnDifference - tDesiredValue; //Gets the dervative error
    
    tDerivative = tError - tPrevError;

    tPrevError = tError;

    double turnMotorPower = tError + tDerivative;

    //tTotalError += tError

    Motor1.spin(forward, lateralMotorPower - turnMotorPower, voltageUnits::volt); //Changes 
    Motor2.spin(forward, lateralMotorPower - turnMotorPower, voltageUnits::volt);
    Motor3.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);
    Motor1.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);
    

    prevError = error;
    tPrevError = tError;

    vex::task::sleep(20);
  }

  return 1;

}

void resetPID(){

  //resets all the PID variables

  error = 0;
  prevError = 0;
  derivative = 0;
  totalError = 0;
  tError = 0;
  tPrevError = 0;
  tDerivative = 0;
  tTotalError = 0;


}



//MARK: Main Function


int main() {

  // Initializing Robot Configuration. DO NOT REMOVE!

  vex::task analysis(analyse); //Using dual tasks for multitasking - the analysis runs seperately at all times.
  vex::task PID(autonPID); 
  //MARK: Setup

  vexcodeInit();
  wait(200, msec);

  while(Inertial20.isCalibrating()) {
    wait(5, msec);
  }

  /*
  
  The autonomous period has been broken into phases, to ensure that collective error does not occur. Collective error 
  occurs when a small error early on can throw off the entire autonomous period.

  Phases also help with our AGILE Development method: allowing us to gradually deploy and push the code instead of the 'Waterfall method', where 
  everything is pushed at the end
  
  */
  //Phase 1
  driveForward(160, 345);
  Brain.Screen.print("Done \n");
  turnLeft(56, 150);
  Brain.Screen.print("Done 2 \n"); //Debugging comments to understand how the the code is working, and indeed if it reaches a point.

  //Phase 2
  driveForward(160, 550);
  Brain.Screen.print("Done 3 \n");
  turnLeft(110, 150);
  driveForward(160, 135);
  wait(500, msec);
  Brain.Screen.print("Done 3 \n");

  
  

  
}