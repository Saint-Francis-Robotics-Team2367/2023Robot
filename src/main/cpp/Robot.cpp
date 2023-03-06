// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc2/command/PIDCommand.h>
#include <iostream>
#include <frc/XboxController.h>

// // All Module Includes
//#include "DriveBaseModule.h"
#include "ScaraArmModule.h"
#include "ElevatorModule.h"

//DriveBaseModule drive;
ScaraArmModule arm = ScaraArmModule();
ElevatorModule elev = ElevatorModule(13);
frc::XboxController* ctr = new frc::XboxController(0);
//cpr number pulses (4096) per rev, 70 to 1 / 360

double arm_startX = arm.innerSize + arm.outterSize;
double arm_startY = 0;

void Robot::RobotInit()
{
  arm.ArmInit();
  arm.innerPID.SetOutputRange(-0.2, 0.2);
  arm.outterPID.SetOutputRange(-0.2, 0.2);
  elev.Init();
  frc::SmartDashboard::PutNumber("X_Calc", arm_startX);
  frc::SmartDashboard::PutNumber("Y_Calc", arm_startY);
  /*
  //compRobotDrive.periodicInit();
  */
  //need drive inits
  //drive.driveThread.detach(); 
 
}

void Robot::RobotPeriodic()
{
  frc::SmartDashboard::PutNumber("Voltage", elev.elevatorMotor->GetBusVoltage());
  frc::SmartDashboard::PutNumber("InnerAngle", arm.inner_enc.GetPosition());
  frc::SmartDashboard::PutNumber("OutterAngle", arm.outter_enc.GetPosition());
  
}
void Robot::AutonomousInit()
{
  // double x = frc::SmartDashboard::GetNumber("X_Calc", arm_startX);
  // double y = frc::SmartDashboard::GetNumber("Y_Calc", arm_startY);
  // arm.movetoXY(x, y);
  

  //drive.state = 'a';
}
void Robot::AutonomousPeriodic()
{
  elev.AutoPeriodic();
}


void Robot::TeleopInit()
{
  //drive.state = 't'; //add codes in while loops to break if state change
}

void Robot::TeleopPeriodic()
{
  arm.inner->Set(ctr->GetRightX() / 5);
  arm.outter->Set(ctr->GetLeftX() / 5);
  elev.TeleopPeriodic(ctr->GetLeftTriggerAxis(), ctr->GetRightTriggerAxis());
  //elev.elevatorMotor->Set(ctr->GetLeftTriggerAxis() - ctr->GetRightTriggerAxis());
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
