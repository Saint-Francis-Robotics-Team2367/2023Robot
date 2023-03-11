// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

//#include <fmt/core.h>

#include "Limelight.h"
#include <frc/smartdashboard/SmartDashboard.h>

//Limelight ll;

void Robot::RobotInit() {}

void Robot::RobotPeriodic() {
  std::vector<double> pose = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumberArray("", std::vector<double>(6));
  frc::SmartDashboard::PutNumber("X", pose.at(0));
  frc::SmartDashboard::PutNumber("Y", pose.at(1));
  frc::SmartDashboard::PutNumber("Angle", pose.at(5));

}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif