// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc2/command/PIDCommand.h>
#include <string>
// // All Module Includes
#include "DriveBaseModule.h"
#include <frc/DigitalInput.h>
#include <rev/CANDigitalInput.h>
//DriveBaseModule drive;
//moved instantiation to h file



void Robot::RobotInit()
{
  //compRobotDrive.periodicInit();

  //need drive inits
  //drive.driveThread.detach();                  
  int test1 = 0; 
  frc::SmartDashboard::PutNumber("test1", ++test1);
  //bool test;
  
}

void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutBoolean("Test", LimitSwitch->Get());
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
 // drive.state = 't'; //add codes in while loops to break if state change
}

void Robot::TeleopPeriodic() {
  
}

void Robot::DisabledInit() {
  //drive.stopAuto = true;
}
void Robot::DisabledPeriodic() {
}

void Robot::TestInit()
{
//  drive.state = 'd';
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
