// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include<frc/XboxController.h>
#include <rev/CANSparkMax.h>


class Robot : public frc::TimedRobot {

double set = 0;
bool isStage = false;
int index = 0;

//rev::CANSparkMax* randomTest = new rev::CANSparkMax(9, rev::CANSparkMax::MotorType::kBrushless);

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

  
};


