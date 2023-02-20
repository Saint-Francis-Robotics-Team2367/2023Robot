// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc2/command/PIDCommand.h>
#include <iostream>

// // All Module Includes
#include "DriveBaseModule.h"

DriveBaseModule drive;

//cpr number pulses (4096) per rev, 70 to 1 / 360

void Robot::RobotInit()
{
  /*
  arm.ArmInit();
  arm.innerPID.SetOutputRange(-0.2, 0.2);
  arm.outterPID.SetOutputRange(-0.2, 0.2);
  frc::SmartDashboard::PutNumber("x", arm.innerSize);
  frc::SmartDashboard::PutNumber("y", arm.outterSize);
  elev.Init();
  //compRobotDrive.periodicInit();
  */
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
}


void Robot::TeleopInit()
{
  drive.state = 't'; //add codes in while loops to break if state change
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
  drive.state = 'd';
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
