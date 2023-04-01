// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

// #include <fmt/core.h>

frc::XboxController *ctr = new frc::XboxController(0);

static const int deviceID = 14;
rev::CANSparkMax m_motor{deviceID, rev::CANSparkMax::MotorType::kBrushless};
rev::SparkMaxPIDController m_pidController = m_motor.GetPIDController();

// Encoder object created to display position values
rev::SparkMaxRelativeEncoder m_encoder = m_motor.GetEncoder();
double kP = 0.1, kI = 1e-4, kD = 1, kIz = 0, kFF = 0, kMaxOutput = 1, kMinOutput = -1, iAccum = 0.1;
// cpr number pulses (4096) per rev, 70 to 1 / 360

void Robot::RobotInit()
{
  m_motor.SetSmartCurrentLimit(30);
  m_encoder.SetPosition(0);
  m_pidController.SetReference(0,rev::ControlType::kPosition);
  // set PID coefficients
  m_pidController.SetP(kP);
  m_pidController.SetI(kI);
  m_pidController.SetD(kD);
  m_pidController.SetIZone(kIz);
  m_pidController.SetFF(kFF);
  m_pidController.SetIMaxAccum(iAccum);
  m_pidController.SetOutputRange(kMinOutput, kMaxOutput);

  // display PID coefficients on SmartDashboard
  frc::SmartDashboard::PutNumber("P Gain", kP);
  frc::SmartDashboard::PutNumber("I Gain", kI);
  frc::SmartDashboard::PutNumber("D Gain", kD);
  frc::SmartDashboard::PutNumber("I Zone", kIz);
  frc::SmartDashboard::PutNumber("Feed Forward", kFF);
  frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
  frc::SmartDashboard::PutNumber("Min Output", kMinOutput);
  frc::SmartDashboard::PutNumber("Set Rotations", 0);
  frc::SmartDashboard::PutNumber("Max accum",0);
  
}

void Robot::RobotPeriodic()
{
}

void Robot::AutonomousInit()
{
}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit()
{
}

void Robot::TeleopPeriodic()
{
  static double max_vel = 0;


  // read PID coefficients from SmartDashboard
  double p = frc::SmartDashboard::GetNumber("P Gain", 0);
  double i = frc::SmartDashboard::GetNumber("I Gain", 0);
  double d = frc::SmartDashboard::GetNumber("D Gain", 0);
  double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
  double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
  double max = frc::SmartDashboard::GetNumber("Max Output", 0);
  double min = frc::SmartDashboard::GetNumber("Min Output", 0);
  double rotations = frc::SmartDashboard::GetNumber("Set Rotations", 0);
  double iaccum = frc::SmartDashboard::GetNumber("Max accum",0);
  // if PID coefficients on SmartDashboard have changed, write new values to controller

  if((iaccum != iAccum))
  {
    iAccum = iaccum;
    m_pidController.SetIMaxAccum(iAccum);
  }
  if ((p != kP))
  {
    m_pidController.SetP(p);
    kP = p;
  }
  if ((i != kI))
  {
    m_pidController.SetI(i);
    kI = i;
  }
  if ((d != kD))
  {
    m_pidController.SetD(d);
    kD = d;
  }
  if ((iz != kIz))
  {
    m_pidController.SetIZone(iz);
    kIz = iz;
  }
  if ((ff != kFF))
  {
    m_pidController.SetFF(ff);
    kFF = ff;
  }
  if ((max != kMaxOutput) || (min != kMinOutput))
  {
    m_pidController.SetOutputRange(min, max);
    kMinOutput = min;
    kMaxOutput = max;
  }
  m_pidController.SetReference(rotations, rev::CANSparkMax::ControlType::kPosition);

  double cur_vel = fabs(m_encoder.GetVelocity());
  if(cur_vel > max_vel)
  {
    max_vel = cur_vel;
  }

  frc::SmartDashboard::PutNumber("SetPoint", rotations);
  frc::SmartDashboard::PutNumber("ProcessVariable", m_encoder.GetPosition());
  frc::SmartDashboard::PutNumber("Velocity", m_encoder.GetVelocity());
  frc::SmartDashboard::PutNumber("Current", m_motor.GetOutputCurrent());
  frc::SmartDashboard::PutNumber("Max Velocity", max_vel);
}

void Robot::DisabledInit()
{
}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif