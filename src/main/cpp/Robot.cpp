// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

//#include <fmt/core.h>

#include "Limelight.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "ScaraArmModule.h"
#include "DriveBaseModule.h"
#include "ElevatorModule.h"
#include <frc/XboxController.h>

frc::XboxController* ctr = new frc::XboxController(0);

ScaraArmModule arm(ctr);
DriveBaseModule drive;
ElevatorModule elev(ctr);
//Limelight ll; 

// #include "ScaraArmModule.h"

// ScaraArmModule arm = ScaraArmModule();
//moved instantiation to h file

//cpr number pulses (4096) per rev, 70 to 1 / 360
//frc::Joystick* driverStick = new frc::Joystick(0);

void Robot::RobotInit()
{
  // arm.ArmInit();
  
  // frc::SmartDashboard::PutNumber("x", arm.innerSize);
  // frc::SmartDashboard::PutNumber("y", arm.outterSize);
  // //compRobotDrive.periodicInit();

  // //need drive inits
  drive.driveThread.detach(); 
  arm.scaraArmThread.detach();
  elev.elevatorThread.detach();
 
}

void Robot::RobotPeriodic()
{
  // frc::SmartDashboard::PutNumber("inner enc", arm.inner_enc.GetPosition());
  // frc::SmartDashboard::PutNumber("outer enc", arm.outter_enc.GetPosition());
  // arm.inner_enc.SetPositionConversionFactor(1);
  // arm.outter_enc.SetPositionConversionFactor(1);
  // double in_angle = arm.inner_enc.GetPosition();
  // double out_angle = arm.outter_enc.GetPosition();
  // frc::SmartDashboard::PutNumber("Inner", in_angle);
  // frc::SmartDashboard::PutNumber("Outter", out_angle);
  // frc::SmartDashboard::PutNumber("XPOS", arm.Angles_to_XY(in_angle, out_angle).at(0));
  // // frc::SmartDashboard::PutNumber("YPOS", arm.Angles_to_XY(in_angle, out_angle).at(1));
  //   if(ctr->GetAButton()) {
  //   frc::SmartDashboard::PutNumber("first ctr1", 20);
  // }

  // // if(ctrextra->GetBButton()) {
  // //   frc::SmartDashboard::PutNumber("second ctr2", 10);
  // // }
  
  // frc::SmartDashboard::PutNumber("trigger", ctr->GetRightTriggerAxis() - ctr->GetLeftTriggerAxis());
  //randomTest->Set(ctr->GetRightTriggerAxis() - ctr->GetLeftTriggerAxis());
}

void Robot::RobotPeriodic() {
  //std::vector<double> pose = ll.getFieldPos();
  //std::vector<double> rra = ll.getRetroreflectiveAngles();
  //int curpipeline = ll.getPipeline();
  //frc::SmartDashboard::PutNumber("Pipeline", curpipeline);
  //frc::SmartDashboard::PutNumber("R1", rra.at(0));
  //frc::SmartDashboard::PutNumber("R2", rra.at(1));

  //Get TargetPose Robot Space:
  // std::vector<double> targetPose = ll.getTargetPoseRobotSpace();
  // Limelight::Point targetXY = ll.getTargetXY(targetPose.at(0) * 39.37, targetPose.at(2) * 39.37, targetPose.at(4), Limelight::bottomLeftPole); // X, Y, yaw, poleID
  // frc::SmartDashboard::PutNumber("TapeX", targetXY.x);
  // frc::SmartDashboard::PutNumber("TapeY", targetXY.y);





}

void Robot::AutonomousInit() {
  arm.state = 'a';
  drive.state = 'a';
  elev.state = 'a';

  // arm.inner_enc.SetPosition(0);
  // arm.outter_enc.SetPosition(0);
  
}
void Robot::AutonomousPeriodic()
{
//  double x = frc::SmartDashboard::GetNumber("x", arm.innerSize);
//  frc::SmartDashboard::PutNumber("x", x);

//  double y = frc::SmartDashboard::GetNumber("y",  arm.outterSize); //CATCH NULL SO VECtor DOeSN'T GO OUT OF RangE!!!!
//  frc::SmartDashboard::PutNumber("y", y);


// arm.movetoXY(x, y);
//arm.movetoXY(35, 15);


//arm.movetoXY(arm.innerSize, arm.outterSize);
//arm.movetoXY(35, 15);
//arm.movetoXY(40, 15);


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