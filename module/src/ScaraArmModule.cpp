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
  // grabber->Init();
  inner_enc.SetPosition(0);
  outter_enc.SetPosition(0);
  inner->SetInverted(false);
  outter->SetInverted(false);
  inner_enc.SetPositionConversionFactor(innerConv);
  outter_enc.SetPositionConversionFactor(outterConv);
  innerPID.SetP(0.32);
  innerPID.SetI(0.00);
  innerPID.SetD(0.0);
  // innerPID.SetIZone(5);
  outterPID.SetP(0.32);
  outterPID.SetI(0.00);
  outterPID.SetD(0.0);
  innerPID.SetOutputRange(-0.2, 0.2);
  outterPID.SetOutputRange(-0.2, 0.2);
}

void ScaraArmModule::stow(double innerSet, double outterSet, double outterSlowSet)
{
  int state = 0;
  float startTime = frc::Timer::GetFPGATimestamp().value();
  float currTime = frc::Timer::GetFPGATimestamp().value();

  while (state < 2)
  {
    if (!isStowing) {
      inner->Set(0);
      outter->Set(0);
      break;
    }
    currTime = frc::Timer::GetFPGATimestamp().value();
    if (currTime - startTime > 4)
    {
      frc::SmartDashboard::PutString("StowStatus", "Went Overtime");
      inner->Set(0);
      outter->Set(0);
      break;
    }
    if (state == 0)
    {
      if (!InnerLimitSwitch.Get())
      {
        inner->Set(0);
        state = 1;
      }
      else
      {
        inner->Set(innerSet);
        outter->Set(-outterSlowSet);
      }
    }
    else if (state == 1)
    {
      if (!OutterLimitSwitch.Get())
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
    // innerPID.SetReference(inner_enc.GetPosition(), rev::CANSparkMax::ControlType::kPosition);
    // outterPID.SetReference(outter_enc.GetPosition(), rev::CANSparkMax::ControlType::kPosition);
  }
  inner->Set(0);
  outter->Set(0);

  
  // Set encoders here if necessary
}

ScaraArmModule::PointXY ScaraArmModule::getPoleXY(Limelight::poleIDs poleID) {
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


  if (currProfile.firstRun) { // Reset profile
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
  // float timeElapsedInner, DistanceToDeccelerateInner, currentVelocityInner = 0.0; //currentPositionInner is the set point
  bool checkPastSetpoint;
  if (currProfile.startPosition > setpoint) {
    checkPastSetpoint = currProfile.currentPosition > setpoint;
  } else {
    checkPastSetpoint = currProfile.currentPosition < setpoint;
  }


  //statePosition
  //if ()
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

      // if(fabs(currProfile.currentPosition) > fabs(setpoint)) {
      //   currProfile.currentPosition = setpoint;
      // }

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
      if (motor == motorMappings::innerMotor) {
        innerPID.SetReference(std::copysign(currProfile.currentPosition, setpoint), rev::CANSparkMax::ControlType::kPosition); // setpoint uses encoder
      } else if (motor == motorMappings::outterMotor) {
        outterPID.SetReference(std::copysign(currProfile.currentPosition, setpoint), rev::CANSparkMax::ControlType::kPosition); // setpoint uses encoder
      }
      currProfile.prevTime = frc::Timer::GetFPGATimestamp().value();
      return false;
    }
  } else {
    currProfile.firstRun = true;
    currProfile.isProfiling = false;
    return true;
  }
}

void ScaraArmModule::run()
{
  runInit();
  isStowing = false;
  bool isPlacing = false;
  PointXY placePoint;
  while (true)
  {
    auto nextRun = std::chrono::steady_clock::now() + std::chrono::milliseconds(5); // change milliseconds at telop
    frc::SmartDashboard::PutNumber("InnerAngle", inner_enc.GetPosition());
    frc::SmartDashboard::PutNumber("OutterAngle", outter_enc.GetPosition());

    if (isStowing)
    {
      stow(0.2, 0.1, 0.05);

      inner_enc.SetPosition(0);
      outter_enc.SetPosition(0);
      innerPID.SetReference(inner_enc.GetPosition() - 15.0, rev::CANSparkMax::ControlType::kPosition);
      outterPID.SetReference(outter_enc.GetPosition() - 15.0, rev::CANSparkMax::ControlType::kPosition);
      

      isStowing = false;
      continue;
    } 
    else if (isPlacing) 
    {
      placePoint = getPoleXY(ll.bottomRightPole);
      //Three states: placing, waiting, going back
      MoveXY::Point target{placePoint.x, placePoint.y};
      armCalc.calc_solution_to_target(target);
      MoveXY::ArmAngles motor_angle = armCalc.get_command_solution();
      frc::SmartDashboard::PutNumber("innerCalc", motor_angle.shoulder);
      frc::SmartDashboard::PutNumber("outterCalc", motor_angle.elbow);
      moveProfiled(motor_angle.elbow, motorMappings::outterMotor);
      moveProfiled(motor_angle.shoulder, motorMappings::innerMotor);


      // outterPID.SetReference(motor_angle.elbow, rev::CANSparkMax::ControlType::kPosition);
      // innerPID.SetReference(motor_angle.shoulder, rev::CANSparkMax::ControlType::kPosition);
    }
    else if (state == 't')
    {
      if (ctrOperator->GetYButtonPressed())
      {
        isStowing = true;
      }
      else if (ctrOperator->GetXButtonPressed())
      {
        innerPID.SetReference(-45, rev::CANSparkMax::ControlType::kPosition);
        outterPID.SetReference(-35, rev::CANSparkMax::ControlType::kPosition);
      }
      else if (ctrOperator->GetAButtonPressed())
      {
        MoveXY::Point target{innerSize, outterSize};
        armCalc.calc_solution_to_target(target);
        MoveXY::ArmAngles motor_angle = armCalc.get_command_solution();
        frc::SmartDashboard::PutNumber("innerCalc", motor_angle.shoulder);
        frc::SmartDashboard::PutNumber("outterCalc", motor_angle.elbow);
        outterPID.SetReference(motor_angle.elbow, rev::CANSparkMax::ControlType::kPosition);
        innerPID.SetReference(motor_angle.shoulder, rev::CANSparkMax::ControlType::kPosition);
      }
      else if (ctrOperator->GetBackButtonPressed()) 
      {
        PointXY poleXY = getPoleXY(ll.bottomRightPole);
        MoveXY::Point target{poleXY.x, poleXY.y};
        armCalc.calc_solution_to_target(target);
        MoveXY::ArmAngles motor_angle = armCalc.get_command_solution();
        frc::SmartDashboard::PutNumber("innerCalc", motor_angle.shoulder);
        frc::SmartDashboard::PutNumber("outterCalc", motor_angle.elbow);
        outterPID.SetReference(motor_angle.elbow, rev::CANSparkMax::ControlType::kPosition);
        innerPID.SetReference(motor_angle.shoulder, rev::CANSparkMax::ControlType::kPosition);
      } 
      else if (ctrOperator->GetStartButton()) 
      {
        moveProfiled(-10.0, motorMappings::outterMotor);
      } 
      else if (ctrOperator->GetLeftBumperPressed()) 
      {
        isPlacing = true;
      }
      else
      {
        // innerPID.SetReference()
        // innerPID.SetReference(inner_enc.GetPosition(),  rev::CANSparkMax::ControlType::kPosition);
        // outterPID.SetReference(outter_enc.GetPosition(),  rev::CANSparkMax::ControlType::kPosition);
      }
    }

    if (state == 'a')
    {
      //grabber->grabberMotor->Set(0.01);
      stow(0.2, 0.1, 0.05);

    }

    if (state == 'd')
    {
      isStowing = false;
    }

    std::this_thread::sleep_until(nextRun);
  }
}
