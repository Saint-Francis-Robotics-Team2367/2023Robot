// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <rev/SparkMaxLimitSwitch.h>
#include <frc/DigitalInput.h>
class Robot : public frc::TimedRobot {



 public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;
  //rev::SparkMaxLimitSwitch * limitSwitch = new rev::SparkMaxLimitSwitch();
  frc::DigitalInput * LimitSwitch = new frc::DigitalInput(0);
};


