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
        if (InnerLimitSwitch.Get()) {
          inner->Set(0);
          state = 1;
        } else {
          inner->Set(-innerSet);
          outter->Set(outterSlowSet);
        }

    } else if (state == 1) {
      if(OutterLimitSwitch.Get()) {
        outter->Set(0);
        state = 2;
        frc::SmartDashboard::PutString("StowStatus", "Finished!");
      } else {
        outter->Set(-outterSet);
      }
    }

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

    if (isStowing) {
      stow(0.2, 0.1, 0.05);
    }
    if (state == 't')
    {
    }

    if (state == 'a')
    {
    }

    if (state == 'd')
    {
    }

    std::this_thread::sleep_until(nextRun);
  }
}
