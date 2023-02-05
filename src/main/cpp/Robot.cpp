// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc2/command/PIDCommand.h>
#include <iostream>

// // All Module Includes
#include "DriveBaseModule.h"

DriveBaseModule drive;
//moved instantiation to h file


#include "ShuffleUI.h"


void Robot::RobotInit()
{
  //compRobotDrive.periodicInit();

  //need drive inits
  drive.driveThread.detach(); 
 
}

void Robot::RobotPeriodic()
{
}
void Robot::AutonomousInit()
{
  drive.state = 'a';
}
void Robot::AutonomousPeriodic()
{
  // testLeftMotor->Set(0.2);
  // testRightMotor->Set(0.2);
}


void Robot::TeleopInit()
{
  drive.state = 't';
}

void Robot::TeleopPeriodic()
{

}

void Robot::DisabledInit() {
  drive.stopAuto = true;
}
void Robot::DisabledPeriodic() {
}

void Robot::TestInit()
{
  drive.state = 'u';
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
