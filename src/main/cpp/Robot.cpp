// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

//#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "ScaraArmModule.h"
#include "DriveBaseModule.h"
#include "ElevatorModule.h"
#include <frc/XboxController.h>

frc::XboxController* ctr = new frc::XboxController(0);
frc::XboxController* ctr2 = new frc::XboxController(1);

ScaraArmModule arm(ctr, ctr2);
DriveBaseModule drive; //joystick is initialized in drive itself (different class of joystick - will fix later...)
ElevatorModule elev(ctr, ctr2);
//Limelight ll; 

//cpr number pulses (4096) per rev, 70 to 1 / 360

void Robot::RobotInit()
{
  drive.driveThread.detach(); 
  arm.scaraArmThread.detach();
  elev.elevatorThread.detach();
}

void Robot::RobotPeriodic() {
  //Get TargetPose Robot Space:
  // std::vector<double> targetPose = ll.getTargetPoseRobotSpace();
  // Limelight::Point targetXY = ll.getTargetXY(targetPose.at(0) * 39.37, targetPose.at(2) * 39.37, targetPose.at(4), Limelight::bottomLeftPole); // X, Y, yaw, poleID
  // frc::SmartDashboard::PutNumber("TapeX", targetXY.x);
  // frc::SmartDashboard::PutNumber("TapeY", targetXY.y);
  // frc::SmartDashboard::PutNumber("Detected?", ll.getTargetDetected());
}

void Robot::AutonomousInit() {
  arm.state = 'a';
  drive.state = 'a';
  elev.state = 'a';
}
void Robot::AutonomousPeriodic() {}


void Robot::TeleopInit() {
  arm.state = 't';
  drive.state = 't';
  elev.state = 't';
}

void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {
}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif