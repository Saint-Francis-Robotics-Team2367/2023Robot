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
  inner->SetInverted(true);
  outter->SetInverted(true);
  inner_enc.SetPositionConversionFactor(innerConv);
  outter_enc.SetPositionConversionFactor(outterConv);
  innerPID.SetP(0.32);
  innerPID.SetI(0.00);
  innerPID.SetD(0.0);
  // innerPID.SetIZone(5);
  outterPID.SetP(0.32);
  outterPID.SetI(0.00);
  outterPID.SetD(0.0);
  innerPID.SetOutputRange(-0.1, 0.1);
  outterPID.SetOutputRange(-0.1, 0.1);

}

void ScaraArmModule::stow(double innerSet, double outterSet, double outterSlowSet) {
  int state = 0;
  float startTime = frc::Timer::GetFPGATimestamp().value();
  float currTime = frc::Timer::GetFPGATimestamp().value();

  while (state < 2) {
    currTime = frc::Timer::GetFPGATimestamp().value();
    if (currTime - startTime > 4) {
      frc::SmartDashboard::PutString("StowStatus", "Went Overtime");
      inner_enc.SetPosition(0);
      outter_enc.SetPosition(0);
      innerPID.SetReference(inner_enc.GetPosition(), rev::CANSparkMax::ControlType::kPosition);
      outterPID.SetReference(outter_enc.GetPosition(), rev::CANSparkMax::ControlType::kPosition);
      break;
    }
    if (state == 0) {
        if (!InnerLimitSwitch.Get()) {
          inner->Set(0);
          state = 1;
        } else {
          inner->Set(-innerSet);
          outter->Set(outterSlowSet);
        }

    } else if (state == 1) {
      if(!OutterLimitSwitch.Get()) {
        outter->Set(0);
        state = 2;
        frc::SmartDashboard::PutString("StowStatus", "Finished!");
      } else {
        outter->Set(-outterSet);
      }
    }
    inner_enc.SetPosition(0);
    outter_enc.SetPosition(0);

  }
  //Set encoders here if necessary
}



void ScaraArmModule::PIDTuning() {
  std::string tab = "arm PID";
  //maybe have input determine if it tunes inner or outer?

  // PID Inner
  double innerOutput = inner->GetOutputCurrent();
  double innerVoltage = inner->GetBusVoltage();

  ShuffleUI::MakeWidget("inner current", tab, innerOutput);
  ShuffleUI::MakeWidget("inner voltage", tab, innerVoltage);

  ShuffleUI::MakeSlider("P value inner", tab, 5, 5, 0);
  innerPID.SetP(ShuffleUI::GetDouble("P value inner", tab, 0));

  ShuffleUI::MakeSlider("I value inner", tab, 5, 5, 0);
  innerPID.SetI(ShuffleUI::GetDouble("I value inner", tab, 0));

  ShuffleUI::MakeSlider("D value", tab, 5, 5, 0);
  innerPID.SetD(ShuffleUI::GetDouble("P value", tab, 0));

  ShuffleUI::MakeSlider("I Zone inner", tab, 5, 5, 0);
  innerPID.SetIZone(ShuffleUI::GetDouble("I Zone inner", tab, 0));

  //so this line is a quick fix for a potential error. I want to fix this by changing the ShuffleUI code, but for now this should work
  if (ShuffleUI::GetEntry("wait time", tab) == NULL) {ShuffleUI::MakeWidget("wait time", tab, 4);}
  double waitTime = ShuffleUI::GetDouble("wait time", tab, 4);
  ShuffleUI::MakeWidget("wait time", tab, waitTime);

  double currTime = frc::Timer::GetFPGATimestamp().value();
  ShuffleUI::MakeWidget("currTime", tab, currTime);
  ShuffleUI::MakeWidget("Setpoint", tab, delta);
  if (currTime > tuningPrevTime + waitTime)
  {

    innerPID.SetReference(delta, rev::ControlType::kPosition);
    delta = delta * -1.0;

    ShuffleUI::MakeWidget("Right Encoder", tab, inner_enc.GetPosition());

    tuningPrevTime = frc::Timer::GetFPGATimestamp().value();
    ShuffleUI::MakeWidget("prev time", tab, tuningPrevTime);
  }


  // PID Outer
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
        innerPID.SetReference(111, rev::CANSparkMax::ControlType::kPosition);
        outterPID.SetReference(45, rev::CANSparkMax::ControlType::kPosition);

      }
      else if (ctrOperator->GetAButtonPressed())
      {
        MoveXY::Point target{innerSize, outterSize};
        armCalc.calc_solution_to_target(target);
        MoveXY::ArmAngles motor_angle = armCalc.get_command_solution();
        frc::SmartDashboard::PutNumber("innerCalc", motor_angle.shoulder);
        frc::SmartDashboard::PutNumber("outterCalc", motor_angle.elbow);
        //outterPID.SetReference(motor_angle.elbow, rev::CANSparkMax::ControlType::kPosition);
        //innerPID.SetReference(motor_angle.shoulder, rev::CANSparkMax::ControlType::kPosition);


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
