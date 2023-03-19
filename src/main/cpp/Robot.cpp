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
DriveBaseModule drive;
ElevatorModule elev(ctr, ctr2);


//cpr number pulses (4096) per rev, 70 to 1 / 360

void Robot::RobotInit()
{
  drive.driveThread.detach(); 
  arm.scaraArmThread.detach();
  elev.elevatorThread.detach();
}

void Robot::RobotPeriodic() {

}

void Robot::AutonomousInit() {
  arm.state = 'a';
  drive.state = 'a';
  elev.state = 'a';
}
void Robot::AutonomousPeriodic() {
  arm.autoX = arm.currentPosition.armX;
  arm.autoY = arm.currentPosition.armY;
}


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