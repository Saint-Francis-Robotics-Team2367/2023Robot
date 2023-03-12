// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

//#include <fmt/core.h>

#include "Limelight.h"
#include <frc/smartdashboard/SmartDashboard.h>

Limelight ll; 

void Robot::RobotInit() {
  ll.switchToPipeline(3);
}

void Robot::RobotPeriodic() {
  std::vector<double> pose = ll.getFieldPos();
  std::vector<double> rra = ll.getRetroreflectiveAngles();
  int curpipeline = ll.getPipeline();
  frc::SmartDashboard::PutNumber("Pipeline", curpipeline);
  frc::SmartDashboard::PutNumber("R1", rra.at(0));
  frc::SmartDashboard::PutNumber("R2", rra.at(1));
  std::vector<double> pose = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumberArray("targetpose_robotspace", std::vector<double>(6));
  frc::SmartDashboard::PutNumber("X", pose.at(0) * 39.37);
  frc::SmartDashboard::PutNumber("Y", pose.at(2) * 39.37);
  frc::SmartDashboard::PutNumber("Angle", pose.at(4));
  std::vector<double> tagXY = ll.getTargetXY(pose.at(0) * 39.37, pose.at(2) * 39.37, pose.at(4), 0);
  frc::SmartDashboard::PutNumber("tagX", tagXY.at(0));
  frc::SmartDashboard::PutNumber("tagY", tagXY.at(1));



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