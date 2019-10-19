#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "v5.h"
#include "v5_vcs.h"
#include "v5_api.h"
#include "vex_competition.h"
#include "vex_controller.h"
#include "vex_global.h"
#include "vex_units.h"

using namespace vex;

vex::brain Brain;
vex::competition Competition;
vex::controller Controller1(controllerType::primary);
vex::motor R1(PORT8,true);
vex::motor R2(PORT9,false);
vex::motor L1(PORT15,false);
vex::motor L2(PORT18,true);
vex::motor Lift1(PORT1,false);  //Right from back
vex::motor Lift2(PORT10,true);  //Left from back
vex::motor Intake(PORT6,true);
vex::motor IntakePivot(PORT2,true);

bool liftHolding;
bool intakeHolding;


void localSleep(int ms){this_thread::sleep_for(ms);}

void sleep(int ms){task::sleep(ms);}

void liftHold(){
  Lift1.stop(brakeType::hold);
  Lift2.stop(brakeType::hold);
  Controller1.Screen.print("       Holding  ");
}

void liftRelease(){
  Lift1.stop(brakeType::coast);
  Lift2.stop(brakeType::coast);
  Controller1.Screen.print("     Not Holding");
}

void arcadeDT(){
  R1.spin(directionType::fwd,Controller1.Axis3.value()-Controller1.Axis4.value(),percentUnits::pct);
  R2.spin(directionType::fwd,Controller1.Axis3.value()-Controller1.Axis4.value(),percentUnits::pct);
  L1.spin(directionType::fwd,Controller1.Axis3.value()+Controller1.Axis4.value(),percentUnits::pct);
  L2.spin(directionType::fwd,Controller1.Axis3.value()+Controller1.Axis4.value(),percentUnits::pct);
}

void lift(){
  if(Controller1.Axis2.value() == 0){
    liftHold();
    liftHolding = true;
  }
  else if(Controller1.Axis2.value() > 0){
    if(liftHolding){
      liftRelease();
      liftHolding = false;
    }

    if(Controller1.ButtonR2.pressing()){
      Lift1.spin(directionType::fwd,Controller1.Axis2.value()/4,percentUnits::pct);
      Lift2.spin(directionType::fwd,Controller1.Axis2.value()/4,percentUnits::pct);
    }
    else{
      Lift1.spin(directionType::fwd,Controller1.Axis2.value()/1.5,percentUnits::pct);
      Lift2.spin(directionType::fwd,Controller1.Axis2.value()/1.5,percentUnits::pct);
    }
  }
  else if(Controller1.Axis2.value() < 0){

    if(liftHolding){
      liftRelease();
      liftHolding = false;
    }

    Lift1.spin(directionType::fwd,Controller1.Axis2.value()/2,percentUnits::pct);
    Lift2.spin(directionType::fwd,Controller1.Axis2.value()/2,percentUnits::pct);

  }
  else{
    Brain.Screen.print("ERROR");
  } 
}

void intake(){
  if(Controller1.ButtonR1.pressing()){
    if(intakeHolding){
      Intake.stop(brakeType::coast);
      intakeHolding = false;
    }

    Intake.spin(directionType::fwd,100,percentUnits::pct);
  }
  else if (Controller1.ButtonR2.pressing()){
    if(intakeHolding){
      Intake.stop(brakeType::coast);
      intakeHolding = false;
    }

    Intake.spin(directionType::rev,80,percentUnits::pct);
  }
  else{
    Intake.stop(brakeType::hold);
    intakeHolding = true;
  }
}

void pivotIntake(){
  if(Controller1.ButtonL1.pressing()){
    IntakePivot.spin(directionType::fwd,50,velocityUnits::pct);
  }
  else if(Controller1.ButtonL2.pressing()){
    IntakePivot.spin(directionType::rev,50,velocityUnits::pct);
  }
  else{
    IntakePivot.stop(brakeType::hold);
  }
}






/*        auton functions     */

void driveStraight(double spin, int speed){
  R1.rotateFor(spin, rotationUnits::rev, speed, velocityUnits::pct, false);
  R2.rotateFor(spin, rotationUnits::rev, speed, velocityUnits::pct, false);
  L1.rotateFor(spin, rotationUnits::rev, speed, velocityUnits::pct, false);
  L2.rotateFor(spin, rotationUnits::rev, speed, velocityUnits::pct, true);
}

void turn(double left, double right, int speed){
  R1.rotateFor(right, rotationUnits::rev, speed, velocityUnits::pct, false);
  R2.rotateFor(right, rotationUnits::rev, speed, velocityUnits::pct, false);
  L1.rotateFor(left, rotationUnits::rev, speed, velocityUnits::pct, false);
  L2.rotateFor(left, rotationUnits::rev, speed, velocityUnits::pct, false);
}

void lift1(double spin, int speed){   //change when potentiometer added
  Lift1.rotateFor(spin, rotationUnits::rev, speed, velocityUnits::pct, false);
  Lift2.rotateFor(spin, rotationUnits::rev, speed, velocityUnits::pct, true);
}

void tilt(double spin, int speed){
  IntakePivot.rotateFor(spin, rotationUnits::rev, speed, velocityUnits::pct, true);
}

void intakeCube(double spin, int speed){
  Intake.rotateFor(spin, rotationUnits::rev, speed, velocityUnits::pct, true);
}

void preAuton(){

}

void auton(){
  driveStraight(1, 20);
  turn(0, 2.5, 30);  //big square pivot left
  intakeCube(-4, 100);
  driveStraight(-2, 50);
}

void driverControl(){
  while(!false){

    arcadeDT();

    lift();

    intake();

    pivotIntake();
    
    localSleep(20);

  }
}
