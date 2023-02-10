// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc2/command/PIDCommand.h>

// // All Module Includes
//#include "DriveBaseModule.h"

//DriveBaseModule drive;
//moved instantiation to h file


double getAngle(double ir1, double ir2) {
    double d = 8.0;
    double result = atan(d/(abs(ir1-ir2)));
    frc::SmartDashboard::PutNumber("Angle", 90-result*(180/3.14159265398979));
    return (90-result*(180/3.14159265398979));
}
void Robot::RobotInit()
{
  //compRobotDrive.periodicInit();

  //need drive inits
  //drive.driveThread.detach(); 
  
}

void Robot::RobotPeriodic()
{
}
void Robot::AutonomousInit()
{
 // drive.state = 'a';
}
void Robot::AutonomousPeriodic()
{
  // testLeftMotor->Set(0.2);
  // testRightMotor->Set(0.2);
}


void Robot::TeleopInit()
{
  //drive.state = 't'; //add codes in while loops to break if state change
}

void Robot::TeleopPeriodic()
{
  double ir1 = proxSensor1->getCM1();
  double ir2 = proxSensor2->getCM2();
  frc::SmartDashboard::PutNumber("IR1", ir1);
  frc::SmartDashboard::PutNumber("IR2", ir2);
  getAngle(ir1, ir2);
}

void Robot::DisabledInit() {
  //drive.stopAuto = true;
}
void Robot::DisabledPeriodic() {
}

void Robot::TestInit()
{
  //drive.state = 'd';
}

void Robot::TestPeriodic()
{
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif