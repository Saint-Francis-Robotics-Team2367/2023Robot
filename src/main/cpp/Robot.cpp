// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

//#include <fmt/core.h>

#include "Limelight.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "ScaraArmModule.h"
#include <frc/XboxController.h>

frc::XboxController* ctr = new frc::XboxController(0);


ScaraArmModule arm(ctr);
Limelight ll; 

void Robot::RobotInit() {
  //ll.switchToPipeline(3);
  arm.scaraArmThread.detach();
  
}

void Robot::RobotPeriodic() {
  //std::vector<double> pose = ll.getFieldPos();
  //std::vector<double> rra = ll.getRetroreflectiveAngles();
  //int curpipeline = ll.getPipeline();
  //frc::SmartDashboard::PutNumber("Pipeline", curpipeline);
  //frc::SmartDashboard::PutNumber("R1", rra.at(0));
  //frc::SmartDashboard::PutNumber("R2", rra.at(1));

  //Get TargetPose Robot Space:
  std::vector<double> targetPose = ll.getTargetPoseRobotSpace();
  Limelight::Point targetXY = ll.getTargetXY(targetPose.at(0) * 39.37, targetPose.at(2) * 39.37, targetPose.at(4), Limelight::bottomRightPole); // X, Y, yaw, poleID
  frc::SmartDashboard::PutNumber("TapeX", targetXY.x);
  frc::SmartDashboard::PutNumber("TapeY", targetXY.y);





}

void Robot::AutonomousInit() {
  arm.state = 'a';
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  arm.state = 't';

  

}

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