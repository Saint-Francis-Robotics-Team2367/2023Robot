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
  // InnerLimitSwitch.EnableLimitSwitch(false);
  // OutterLimitSwitch.EnableLimitSwitch(false);

  inner_enc.SetPosition(0);
  outter_enc.SetPosition(0);

  inner->SetInverted(false);
  outter->SetInverted(false);

  inner_enc.SetPositionConversionFactor(innerConv);
  outter_enc.SetPositionConversionFactor(outterConv);

  innerPID.SetP(innerP);
  innerPID.SetI(innerI);
  innerPID.SetD(innerD);
  innerPID.SetIZone(innerIZone);
  innerPID.SetIMaxAccum(innerMaxAccum);

  outterPID.SetP(outterP);
  outterPID.SetI(outterI);
  outterPID.SetD(outterD);
  outterPID.SetIZone(outterIZone);

  innerPID.SetOutputRange(-0.6, 0.6);
  outterPID.SetOutputRange(-0.5, 0.5);

  inner->SetSmartCurrentLimit(20);
  outter->SetSmartCurrentLimit(20);

  grabber->Init();
  InnerLimitSwitchForward.EnableLimitSwitch(false);
  OutterLimitSwitchForward.EnableLimitSwitch(false);
  InnerLimitSwitchReverse.EnableLimitSwitch(false);
  OutterLimitSwitchReverse.EnableLimitSwitch(false);


}

void ScaraArmModule::run()
{
  runInit();
  isStowing = false;
  frc::SmartDashboard::PutNumber("XSet", innerSize);
  frc::SmartDashboard::GetNumber("YSet", outterSize);


  while (true)
  {
    auto nextRun = std::chrono::steady_clock::now() + std::chrono::milliseconds(5); // change milliseconds at telop
    frc::SmartDashboard::PutNumber("InnerAngle", inner_enc.GetPosition());
    frc::SmartDashboard::PutNumber("OutterAngle", outter_enc.GetPosition());
    frc::SmartDashboard::PutBoolean("OutterLimitSwitch", OutterLimitSwitchForward.Get());
    frc::SmartDashboard::PutBoolean("InnerLimitSwitch", InnerLimitSwitchForward.Get());

    frc::SmartDashboard::PutNumber("GrabberEnc", grabber->grab_enc.GetPosition());
    frc::SmartDashboard::PutNumber("GrabSwitch", grabber->reverseSwitch.Get());

    if (isStowing)
    {
      stow(0.3, 0.2, 0.1);

      inner_enc.SetPosition(0);
      outter_enc.SetPosition(0);
      innerPID.SetReference(-10.0, rev::CANSparkMax::ControlType::kPosition);
      outterPID.SetReference(-15.0, rev::CANSparkMax::ControlType::kPosition);

      isStowing = false;
      continue;
    }

    else if (state == 't')
    {
      //frc::SmartDashboard::PutNumber("GrabberSetpt", frc::SmartDashboard::GetNumber("GrabberSetpt", 0));
      frc::SmartDashboard::PutNumber("GrabberState", grabber->state);
      if (ctrOperator->GetBButtonReleased()) {
        grabber->togglePID();
      }
      if (ctrOperator->GetStartButtonReleased()) {
        flipJstick = !flipJstick;
      }

      frc::SmartDashboard::PutBoolean("FlipJstick", flipJstick);


      if (ctrOperator->GetYButtonPressed())
      {
        isStowing = true;
      }
       
      else if (ctrOperator->GetXButtonPressed())
      {
        innerPID.SetReference(-30.0, rev::CANSparkMax::ControlType::kPosition);
        outterPID.SetReference(-50.0, rev::CANSparkMax::ControlType::kPosition);
      }

      // else if (ctrOperator->GetAButtonPressed())
      // {
      //   innerPID.SetReference(-10.0, rev::CANSparkMax::ControlType::kPosition);
      //   outterPID.SetReference(-10.0, rev::CANSparkMax::ControlType::kPosition);
      //   inner->SetSmartCurrentLimit(20);
      //   outter->SetSmartCurrentLimit(20);
      // }

      else if (ctrOperator->GetAButton())
      {
        PointXY poleXY = getPoleXY(ll.bottomRightPole);
        MoveXY::Point target{poleXY.x, poleXY.y};
        frc::SmartDashboard::PutNumber("TapeX", poleXY.x);
        frc::SmartDashboard::PutNumber("Tapey", poleXY.y);
        armCalc.calc_solution_to_target(target, 1, inner_enc.GetPosition() + innerCommandOffset);
        MoveXY::ArmAngles motor_angle = armCalc.get_command_solution();
        frc::SmartDashboard::PutNumber("innerCalc", motor_angle.shoulder);
        frc::SmartDashboard::PutNumber("outterCalc", motor_angle.elbow);
        outterPID.SetReference(motor_angle.elbow, rev::CANSparkMax::ControlType::kPosition);
        innerPID.SetReference(motor_angle.shoulder, rev::CANSparkMax::ControlType::kPosition);;
      }

      else if (ctrOperator->GetBackButtonPressed())
      {
        // PointXY poleXY = getPoleXY(ll.bottomRightPole);
        // MoveXY::Point target{poleXY.x, poleXY.y};
        // armCalc.calc_solution_to_target(target);
        // MoveXY::ArmAngles motor_angle = armCalc.get_command_solution();
        // frc::SmartDashboard::PutNumber("innerCalc", motor_angle.shoulder);
        // frc::SmartDashboard::PutNumber("outterCalc", motor_angle.elbow);
        // outterPID.SetReference(motor_angle.elbow, rev::CANSparkMax::ControlType::kPosition);
        // innerPID.SetReference(motor_angle.shoulder, rev::CANSparkMax::ControlType::kPosition);
        innerPID.SetReference(-230.0, rev::CANSparkMax::ControlType::kPosition);
        outterPID.SetReference(-20.0, rev::CANSparkMax::ControlType::kPosition);

        
      }

      else if (ctrOperator->GetRightBumper())
      {
        // inner->Set(ctrOperator->GetLeftX());
        // outter->Set(ctrOperator->GetRightX());
        if (!flipJstick) {
          jstickArmMovement(ctrOperator->GetLeftX(), -ctrOperator->GetLeftY());
        } else {
          jstickArmMovement(-ctrOperator->GetLeftX(), ctrOperator->GetLeftY());
        }
        
      }

      else
      {
      }
    }

    if (state == 'a')
    {
      stow(0.2, 0.1, 0.05);
    }

    if (state == 'd')
    {
      isStowing = false;
    }

    std::this_thread::sleep_until(nextRun);
  }
}

void ScaraArmModule::stow(double innerSet, double outterSet, double outterSlowSet)
{
  int state = 0;
  float startTime = frc::Timer::GetFPGATimestamp().value();
  float currTime = frc::Timer::GetFPGATimestamp().value();

  while (state < 2)
  {
    if (!isStowing)
    {
      inner->Set(0);
      outter->Set(0);
      break;
    }
    currTime = frc::Timer::GetFPGATimestamp().value();
    if (currTime - startTime > 5)
    {
      frc::SmartDashboard::PutString("StowStatus", "Went Overtime");
      inner->Set(0);
      outter->Set(0);
      break;
    }
    if (state == 0)
    {
      if (!InnerLimitSwitchForward.Get())
      {
        inner->Set(0);
        state = 1;
      }
      else
      {
        inner->Set(innerSet);
        outter->Set(outterSlowSet);
      }
    }
    else if (state == 1)
    {
      if (!OutterLimitSwitchForward.Get())
      {
        outter->Set(0);
        state = 2;
        frc::SmartDashboard::PutString("StowStatus", "Finished!");
      }
      else
      {
        outter->Set(outterSet);
      }
    }
  }
  // Set encoders here if necessary
}

ScaraArmModule::PointXY ScaraArmModule::getPoleXY(Limelight::poleIDs poleID)
{
  std::vector<double> targetPose = ll.getTargetPoseRobotSpace();
  Limelight::Point targetXY = ll.getTargetXY(targetPose.at(0) * 39.37, targetPose.at(2) * 39.37, targetPose.at(4), poleID); // X, Y, yaw, poleID
  frc::SmartDashboard::PutNumber("TapeX", targetXY.x);
  frc::SmartDashboard::PutNumber("TapeY", targetXY.y);
  return PointXY{targetXY.x, targetXY.y};
}

bool ScaraArmModule::moveProfiled(double setpoint, motorMappings motor)
{
  armProfile currProfile;
  if (motor == motorMappings::innerMotor)
  {
    currProfile = innerProfile;
    currProfile.currentPosition = inner_enc.GetPosition();
  }
  else if (motor == motorMappings::outterMotor)
  {
    currProfile = outterProfile;
    currProfile.currentPosition = outter_enc.GetPosition();
  }

  if (currProfile.firstRun)
  { // Reset profile
    currProfile.startPosition = currProfile.currentPosition;
    currProfile.DistanceToDeccelerate = 0.0;
    currProfile.timeElapsed = 0.0;
    currProfile.currentVelocity = 0.0;
    currProfile.prevTime = frc::Timer::GetFPGATimestamp().value();
    currProfile.positive = true;
    if (setpoint < currProfile.currentPosition)
    {
      currProfile.positive = false;
    }
    currProfile.firstRun = false;
  }
  bool checkPastSetpoint;
  if (currProfile.startPosition > setpoint)
  {
    checkPastSetpoint = currProfile.currentPosition > setpoint;
  }
  else
  {
    checkPastSetpoint = currProfile.currentPosition < setpoint;
  }

  if (checkPastSetpoint && currProfile.isProfiling)
  {

    if (fabs(currProfile.currentPosition - setpoint) > 0)
    {
      if (stopAuto)
      {
        return true;
      }

      currProfile.timeElapsed = frc::Timer::GetFPGATimestamp().value() - currProfile.prevTime;
      currProfile.DistanceToDeccelerate = (3 * currProfile.currentVelocity * currProfile.currentVelocity) / (2 * maxAcc); // change
      if (fabs(currProfile.DistanceToDeccelerate) > fabs(setpoint - currProfile.currentPosition))
      {
        currProfile.currentVelocity -= (maxAcc * currProfile.timeElapsed);
      }
      else // increase velocity
      {
        currProfile.currentVelocity += (maxAcc * currProfile.timeElapsed);
        if (fabs(currProfile.currentVelocity) > fabs(maxVelocity))
        {
          currProfile.currentVelocity = maxVelocity;
        }
      }

      if (currProfile.positive)
      {
        currProfile.currentPosition += currProfile.currentVelocity * currProfile.timeElapsed;
      }
      else
      {
        currProfile.currentPosition -= currProfile.currentVelocity * currProfile.timeElapsed;
      }
      if (!currProfile.positive)
      {
        if (currProfile.currentPosition < setpoint)
        {
          currProfile.currentPosition = setpoint;
        }
      }
      else
      {
        if (currProfile.currentPosition > setpoint)
        {
          currProfile.currentPosition = setpoint;
        }
      }

      frc::SmartDashboard::PutNumber("currPosOuter", currProfile.currentPosition);

      // include clamps here!!!
      frc::SmartDashboard::PutNumber("setpoint", setpoint);
      if (motor == motorMappings::innerMotor)
      {
        innerPID.SetReference(std::copysign(currProfile.currentPosition, setpoint), rev::CANSparkMax::ControlType::kPosition); // setpoint uses encoder
      }
      else if (motor == motorMappings::outterMotor)
      {
        outterPID.SetReference(std::copysign(currProfile.currentPosition, setpoint), rev::CANSparkMax::ControlType::kPosition); // setpoint uses encoder
      }
      currProfile.prevTime = frc::Timer::GetFPGATimestamp().value();
      return false;
    }
  }
  else
  {
    currProfile.firstRun = true;
    currProfile.isProfiling = false;
    return true;
  }
}

void ScaraArmModule::jstickArmMovement(double jstickX, double jstickY)
{
  MoveXY::Point currXY = armCalc.command_to_xy(MoveXY::ArmAngles{inner_enc.GetPosition() + innerCommandOffset,  outter_enc.GetPosition() + outterCommandOffset});
  currentPosition.armX = currXY.x;
  currentPosition.armY = currXY.y;

  if (XYInRange(currentPosition.armX + (jstickX * teleopFactor), currentPosition.armY + (jstickY * teleopFactor)))
  {
    currentPosition.armX += jstickX * teleopFactor;
    currentPosition.armY += jstickY * teleopFactor;
    ShuffleUI::MakeWidget("SetX", tab, currentPosition.armX);
    ShuffleUI::MakeWidget("SetY", tab, currentPosition.armY);


    armCalc.calc_solution_to_target(MoveXY::Point{currentPosition.armX, currentPosition.armY}, 1, inner_enc.GetPosition() + innerCommandOffset);
    MoveXY::ArmAngles motor_angle = armCalc.get_command_solution();
    frc::SmartDashboard::PutNumber("CalcInner", motor_angle.shoulder);
    frc::SmartDashboard::PutNumber("CalcOutter", motor_angle.elbow);


    outterPID.SetReference(motor_angle.elbow, rev::CANSparkMax::ControlType::kPosition);
    innerPID.SetReference(motor_angle.shoulder, rev::CANSparkMax::ControlType::kPosition);
    armCalc.set_target_to_current();
    ShuffleUI::MakeWidget("Invalid?", tab, 0);

  }
  else
  {
    ShuffleUI::MakeWidget("InnerSet", tab, atan2(jstickY, jstickX) * 180 / PI);
    ShuffleUI::MakeWidget("OutterSet", tab, 0);
    ShuffleUI::MakeWidget("Invalid?", tab, 1);

    //innerPID.SetReference(atan2(jstickY, jstickX) * 180 / PI, rev::ControlType::kPosition);
    //outterPID.SetReference(0, rev::ControlType::kPosition);
  }
}

bool ScaraArmModule::XYInRange(double x, double y)
{
  double circle_x = 0;
  double circle_y = 0;
  double rad = outterSize + innerSize;
  if ((x - circle_x) * (x - circle_x) +
          (y - circle_y) * (y - circle_y) <=
      rad * rad)
    return true;
  else
    return false;
}
