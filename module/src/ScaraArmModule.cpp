#include <cmath>
#include <vector>
#include <iostream>
#include <tuple>
#include "ScaraArmModule.h"
#include <frc/Timer.h>
#include "ShuffleUI.h"

ScaraArmModule::ScaraArmModule(frc::XboxController *controller, frc::XboxController *controllerOperator)
{
  ctr = controller;
  ctrOperator = controllerOperator;
  scaraArmThread = std::thread(&ScaraArmModule::run, this); // initializing thread so can detach in robot init
}

void ScaraArmModule::runInit()
{
  InnerLimitSwitch.EnableLimitSwitch(false);
  OutterLimitSwitch.EnableLimitSwitch(false);
  grabber->Init();
  inner_enc.SetPosition(0);
  outter_enc.SetPosition(0);
  inner_enc.SetPositionConversionFactor(innerConv);
  outter_enc.SetPositionConversionFactor(outterConv);

}

void ScaraArmModule::stow(double innerSet, double outterSet, double outterSlowSet) {
  int state = 0;
  float startTime = frc::Timer::GetFPGATimestamp().value();
  float currTime = frc::Timer::GetFPGATimestamp().value();

  while (state < 2) {
    currTime = frc::Timer::GetFPGATimestamp().value();
    if (currTime - startTime > 2) {
      frc::SmartDashboard::PutString("StowStatus", "Went Overtime");
      break;
    }
    if (state == 0) {
        if (!InnerLimitSwitch.Get()) {
          inner->Set(0);
          state = 1;
        } else {
          inner->Set(innerSet);
          outter->Set(-outterSlowSet);
        }

    } else if (state == 1) {
      if(!OutterLimitSwitch.Get()) {
        outter->Set(0);
        state = 2;
        frc::SmartDashboard::PutString("StowStatus", "Finished!");
      } else {
        outter->Set(outterSet);
      }
    }
    inner_enc.SetPosition(0);
    outter_enc.SetPosition(0);

  }
  //Set encoders here if necessary
}






void ScaraArmModule::run()
{
  runInit();
  bool isStowing = false;
  while (true)
  {
    auto nextRun = std::chrono::steady_clock::now() + std::chrono::milliseconds(5); // change milliseconds at telop
    frc::SmartDashboard::PutNumber("InnerAngle", inner_enc.GetPosition());
    frc::SmartDashboard::PutNumber("OutterAngle", outter_enc.GetPosition());

    if (isStowing) {
      stow(0.2, 0.1, 0.05);
      isStowing = false;
      continue;
    }
    if (state == 't')
    {
      if (ctrOperator->GetYButtonPressed()) 
      {
        isStowing = true;
      }
      else if (ctrOperator->GetXButtonPressed()) 
      {
        innerPID.SetReference(-64.4569, rev::CANSparkMax::ControlType::kPosition);
        outterPID.SetReference(-36.6667, rev::CANSparkMax::ControlType::kPosition);

      }
      else if (ctrOperator->GetAButtonPressed())
      {
        MoveXY::Point target{innerSize, outterSize};
        armCalc.calc_solution_to_target(target);
        MoveXY::ArmAngles motor_angle = armCalc.get_command_solution();
        frc::SmartDashboard::PutNumber("innerCalc", motor_angle.shoulder);
        frc::SmartDashboard::PutNumber("outterCalc", motor_angle.elbow);


      }


    }

    if (state == 'a')
    {
    }

    if (state == 'd')
    {
      isStowing = false;
    }

    std::this_thread::sleep_until(nextRun);
  }
}
