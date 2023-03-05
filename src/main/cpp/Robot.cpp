// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc2/command/PIDCommand.h>

// // All Module Includes
#include "DriveBaseModule.h"

//DriveBaseModule drive;
//moved instantiation to h file

#include <networktables/NetworkTableInstance.h>

void Robot::RobotInit()
{
  //compRobotDrive.periodicInit();

  //need drive inits
  //drive.driveThread.detach(); 
 
}

void Robot::RobotPeriodic()
{
  double px = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0.0);
  double py = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty",0.0);
  // double tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0.0);
  // double tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0.0);

  frc::SmartDashboard::PutNumber("TX", px);
  frc::SmartDashboard::PutNumber("TY", py);

  double nx = (1/160) * (px - 159.5);
  double ny = (1/120) * (119.5 - py);

  double horizontal_fov = 54;
  double vertical_fov = 41;

  double vpw = 2.0*tan(horizontal_fov/2);
  double vph = 2.0*tan(vertical_fov/2);

  double x = vpw/2 * nx;
  double y = vph/2 * ny;

  double ax = atan2(1,x);
  double ay = atan2(1,y);
  
  double axr = (ax * PI)/180;
  double ayr = (ay * PI)/180;

  double d = 4;
  double rx = d * tan(axr);
  double ry = sqrt(d*d + rx*rx) * tan(ayr);

  frc::SmartDashboard::PutNumber("AX", ax);
  frc::SmartDashboard::PutNumber("AY", ay);
  frc::SmartDashboard::PutNumber("RX", rx);
  frc::SmartDashboard::PutNumber("RY", ry);
}
void Robot::AutonomousInit()
{
  //drive.state = 'a';
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
