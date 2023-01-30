// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc2/command/PIDCommand.h>

// // All Module Includes
//#include "DriveBaseModule.h"

//DriveBaseModule drive;

#include "ScaraArmModule.h"
#include<frc/XboxController.h>
ScaraArmModule arm = ScaraArmModule();
//moved instantiation to h file

frc::XboxController* ctr = new frc::XboxController(0);

void Robot::RobotInit()
{
  arm.ArmInit();
  frc::SmartDashboard::PutNumber("x", 0);
  frc::SmartDashboard::PutNumber("x", 0);
  //compRobotDrive.periodicInit();

  //need drive inits
  //drive.driveThread.detach(); 
 
}

void Robot::RobotPeriodic()
{
  double in_angle = arm.inner_enc.GetPosition();
  double out_angle = arm.outter_enc.GetPosition();
  frc::SmartDashboard::PutNumber("Inner", in_angle);
  frc::SmartDashboard::PutNumber("Outter", out_angle);  
  frc::SmartDashboard::PutNumber("XPOS", arm.Angles_to_XY(in_angle, out_angle).at(0));
  frc::SmartDashboard::PutNumber("YPOS", arm.Angles_to_XY(in_angle, out_angle).at(1));
  
}
void Robot::AutonomousInit()
{
  //drive.state = 'a';
}
void Robot::AutonomousPeriodic()
{
  double x = frc::SmartDashboard::GetNumber("x", 0);
  double y = frc::SmartDashboard::GetNumber("y", 0);
  if (ctr->GetAButton()) {
    arm.movetoXY(x, y);
  }
  if (ctr->GetBButton()) {
    arm.inner->Set(0);
    arm.outter->Set(0);
  }
  frc::SmartDashboard::PutNumber("x", x);
  frc::SmartDashboard::PutNumber("y", y);



  // testLeftMotor->Set(0.2);
  // testRightMotor->Set(0.2);
}


void Robot::TeleopInit()
{
  

  //drive.state = 't'; //add codes in while loops to break if state change
}

void Robot::TeleopPeriodic()
{
  arm.inner->Set(ctr->GetLeftY() / 4);
  arm.outter->Set(ctr->GetRightY() / 4);
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
