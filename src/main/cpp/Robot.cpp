// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc2/command/PIDCommand.h>

// // All Module Includes
#include "DriveBaseModule.h"
#include "ElevatorModule.h"
#include "ScaraArmModule.h"

frc::XboxController* ctr = new frc::XboxController(0); 

DriveBaseModule drive;
ElevatorModule elev(ctr);
ScaraArmModule arm(ctr);

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
  //drive.driveThread.detach(); 
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
  // frc::SmartDashboard::PutNumber("YPOS", arm.Angles_to_XY(in_angle, out_angle).at(1));
  
}
void Robot::AutonomousInit()
{
  drive.state = 'a';
  elev.state = 'a';
  arm.state = 'a';
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


void Robot::TeleopInit()
{
  drive.state = 't'; //add codes in while loops to break if state change
  elev.state = 't';
  arm.state = 't';
  //frc::SmartDashboard::PutNumber("setpoint", 0);
}

void Robot::TeleopPeriodic()
{
//   frc::SmartDashboard::PutNumber("left y", ctr->GetLeftY());
//   arm.inner->Set(ctr->GetLeftY() / 5);
//   //arm.innerPID.SetReference(10, rev::CANSparkMax::ControlType::kPosition);
//   //arm->innerPID.SetReference(10, rev::CANSparkMax::ControlType::kPosition);
//   frc::SmartDashboard::PutNumber("right y", ctr->GetRightY());
//  arm.outter->Set(ctr->GetRightY()/ 5);
  

//   //arm.innerPID.SetReference(set, rev::CANSparkMax::ControlType::kPosition);
  

//   frc::SmartDashboard::PutNumber("Encoder Pos", arm.outter_enc.GetPosition());
//   frc::SmartDashboard::PutNumber("Voltage/PID Output", arm.outter->GetBusVoltage());
  //current pos, setpoint, outputPID, voltage
 //arm.inner->Set(0.4);
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
