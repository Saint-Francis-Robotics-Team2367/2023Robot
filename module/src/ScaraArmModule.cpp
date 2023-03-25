#include <cmath>
#include <vector>
#include <iostream>
#include <tuple>
#include "ScaraArmModule.h"
#include <frc/Timer.h>
#include "ShuffleUI.h"

ScaraArmModule::ScaraArmModule(frc::XboxController* controller, frc::XboxController* controllerOperator) {
  ctr = controller;
  ctrOperator = controllerOperator;
  scaraArmThread = std::thread(&ScaraArmModule::run, this); //initializing thread so can detach in robot init


}

void ScaraArmModule::ArmInit() {
  inner_enc.SetPositionConversionFactor(innerConv);
  outter_enc.SetPositionConversionFactor(outterConv);
  inner->SetInverted(false);
  outter->SetInverted(false);
  inner_enc.SetPosition(0);
  outter_enc.SetPosition(0);
  inner->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  outter->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  inner_enc.SetPosition(startInner);
  outter_enc.SetPosition(startOutter);
  innerPID.SetP(0.32);
  innerPID.SetI(0.00);
  innerPID.SetD(0.0);
  // innerPID.SetIZone(5);
  outterPID.SetP(0.32);
  outterPID.SetI(0.00);
  outterPID.SetD(0.0);
  outterPID.SetIZone(0.0);

  inner->SetSmartCurrentLimit(30);
  outter->SetSmartCurrentLimit(20);
  //inner->SetSecondaryCurrentLimit(35, )

  innerPID.SetOutputRange(-0.2, 0.2);
  outterPID.SetOutputRange(-0.2, 0.2);
  
  currentPosition.inner_angle = inner_enc.GetPosition();
  currentPosition.outter_angle = outter_enc.GetPosition();
  //std::vector<double> curr_xy = Angles_to_XY(inner_enc.GetPosition(), outter_enc.GetPosition());
  currentPosition.armX  = 0;
  currentPosition.armY  = 0;

  // if (XYInRange(curr_xy.at(0), curr_xy.at(1))) {
  //   currentPosition.armX = curr_xy.at(0);
  //   currentPosition.armY = curr_xy.at(1);
  // } else {
  //   ShuffleUI::MakeWidget("Init Failed", tab, true);
  //   //armTab.AddBoolean("Init Failed", true);
  // }
}

void ScaraArmModule::stow() {
    int stageOutter = 0;
    int stageInner = 0;
    float startTime = frc::Timer::GetFPGATimestamp().value();
    
    while (!(stageInner == 2 && stageOutter == 2)) {
      if(frc::Timer::GetFPGATimestamp().value() - startTime > 3) {
        ShuffleUI::MakeWidget("TimeUp", tab, 1);
        break;
      }
      ShuffleUI::MakeWidget("InnerSwitch", tab, InnerLimitSwitch.Get());
      ShuffleUI::MakeWidget("OutterSwitch", tab, OutterLimitSwitch.Get());

      if (stageOutter == 0) {
        if (OutterLimitSwitch.Get() == false) {
          stageOutter = 1;

        } else {
          outter->Set(0.1);
        }
      }
      if (stageOutter == 1) {
        outter->Set(-0.05);
        if (OutterLimitSwitch.Get() == true) {
          stageOutter = 2;
        }
        
      } 
      if (stageOutter == 2) {
        outter->Set(0);
      }
      
      //Inner
      if (stageInner == 0) {
        if (InnerLimitSwitch.Get() == false) {
          stageInner = 1;

        } else {
          inner->Set(0.15);
        }
      }
      if (stageInner == 1) {
        inner->Set(-0.1);
        if (InnerLimitSwitch.Get() == true) {
          stageInner = 2;
        }
        
      } 
      if (stageInner == 2) {
        inner->Set(0);
      }
      

    }
    inner_enc.SetPosition(startInner);
    outter_enc.SetPosition(startOutter);
}




/*
void ScaraArmModule::ArmPeriodic() {
  double sensor_position = lArmEncoder.GetPosition();
  double theta = plot_point({43,25}, 30, 30);
  cout << theta;
  if (sensor_position>theta) {
    double p = frc::SmartDashboard::GetNumber("P Gain", 0);
    double i = frc::SmartDashboard::GetNumber("I Gain", 0);
    double d = frc::SmartDashboard::GetNumber("D Gain", 0);
    if((p != kp)) {
      lArmPID.SetP(p);
      rArmPID.SetP(p);
      kp = p;
    }
    if((i != ki)) {
      lArmPID.SetI(i);
      rArmPID.SetP(i);
      ki = i;
    }
    if((d != kd)) {
      lArmPID.SetD(d);
      rArmPID.SetP(d);
      kd = d;
    }
  }
}
*/


/*
std::vector<double> ScaraArmModule::Angles_to_XY(double innerAngleDeg, double outterAngleDeg) {
  std::vector<double> output = {};
  double x1 =  innerSize * cos(innerAngleDeg * M_PI / 180);
  double y1 = innerSize * sin(innerAngleDeg * M_PI / 180);
  double x2 = x1 + (outterSize * cos((outterAngleDeg * M_PI / 180) + innerAngleDeg * M_PI / 180));
  double y2 = y1 + (outterSize * sin((outterAngleDeg * M_PI / 180) + innerAngleDeg * M_PI / 180));

  output.push_back(x2);
  output.push_back(y2);
  return output;
}
*/

void ScaraArmModule::runInit() {
  InnerLimitSwitch.EnableLimitSwitch(false);
  OutterLimitSwitch.EnableLimitSwitch(false);
  ArmInit();
  grabber->Init();
  grabber->grabberMotor->SetSmartCurrentLimit(1);
  armCalc.m_curr_elbow_xy.x = 0;
  armCalc.m_curr_elbow_xy.y = 0;

}

void ScaraArmModule::jstickArmMovement(double jstickX, double jstickY) {
  double factor = 1;
  innerPID.SetOutputRange(-0.3, 0.3);
  outterPID.SetOutputRange(-0.2, 0.2);
  currentPosition.armX += jstickX * factor;
  currentPosition.armY += jstickY * factor;
  ShuffleUI::MakeWidget("SetX", tab, currentPosition.armX);
  ShuffleUI::MakeWidget("SetY", tab, currentPosition.armY);
  
  MoveXY::Point target (currentPosition.armX, currentPosition.armY);
  armCalc.calc_solution_to_target(target, 0);

  MoveXY::ArmAngles motor_angle = armCalc.get_command_solution();
  
  
  innerPID.SetReference(motor_angle.shoulder, rev::CANSparkMax::ControlType::kPosition);
  outterPID.SetReference(motor_angle.elbow, rev::CANSparkMax::ControlType::kPosition);
  armCalc.set_target_to_current();
}


void ScaraArmModule::movetoPole(Limelight::poleIDs poleID) {
  std::vector<double> targetPose = ll.getTargetPoseRobotSpace();
  Limelight::Point targetXY = ll.getTargetXY(targetPose.at(0) * 39.37, targetPose.at(2) * 39.37, targetPose.at(4), poleID); // X, Y, yaw, poleID
  ShuffleUI::MakeWidget("TapeX", tab, targetXY.x);
  ShuffleUI::MakeWidget("TapeY", tab, targetXY.y);
  //frc::SmartDashboard::PutNumber("Detected?", ll.getTargetDetected());
  //if (ll.getTargetDetected()) {
  MoveXY::Point target (targetXY.x, targetXY.y);
  if (poleID == Limelight::bottomLeftPole || poleID == Limelight::topLeftPole) {
    armCalc.calc_solution_to_target(target, 2);
  }
  else if (poleID == Limelight::bottomRightPole || poleID == Limelight::topRightPole) {
    armCalc.calc_solution_to_target(target, 1);
  }

  MoveXY::ArmAngles motor_angle = armCalc.get_command_solution();
  
  
  innerPID.SetReference(motor_angle.shoulder, rev::CANSparkMax::ControlType::kPosition);
  outterPID.SetReference(motor_angle.elbow, rev::CANSparkMax::ControlType::kPosition);
  armCalc.set_target_to_current();
  
}


double ScaraArmModule::deadZoneCtr(double input) {
  if (fabs(input) < 0.1) {
    return 0;
  } else {
    return input;
  }
}

void ScaraArmModule::run(){
   runInit();
   test = true;
   
   bool teleopInit = true; //Run the init if condition in the teleop if condition
   bool setManualXY = true;
   bool setLLXY = true;
   int teleopMode = 0;

   int counter = 0;
    while(true) {
      frc::SmartDashboard::PutNumber("grabberStuff", grabber->grabSwitch.Get());
        auto nextRun = std::chrono::steady_clock::now() + std::chrono::milliseconds(5); //change milliseconds at telop
        ShuffleUI::MakeWidget("scara arm module", tab, true);
        ShuffleUI::MakeWidget("InnerAngle", tab, inner_enc.GetPosition());
        ShuffleUI::MakeWidget("OutterAngle", tab, outter_enc.GetPosition());
        ShuffleUI::MakeWidget("InnerSwitch", tab, InnerLimitSwitch.Get());
        ShuffleUI::MakeWidget("OutterSwitch", tab, OutterLimitSwitch.Get());


        if(state == 't') {
          if (teleopInit) {
            inner_enc.SetPosition(0);
            outter_enc.SetPosition(0);
            innerPID.SetReference(0, rev::CANSparkMax::ControlType::kPosition);
            outterPID.SetReference(0, rev::CANSparkMax::ControlType::kPosition);
            teleopInit = false;
          }

          grabber->toggleTwo(ctrOperator->GetBButtonPressed()); //Only Open/Close

          ShuffleUI::MakeWidget("Grabber", tab, grabber->grab_enc.GetPosition());
          ShuffleUI::MakeWidget("InnerAngle", tab, inner_enc.GetPosition());
          ShuffleUI::MakeWidget("OutterAngle", tab, outter_enc.GetPosition());
          ShuffleUI::MakeWidget("TeleopMode", tab, teleopMode);

          if (ctrOperator->GetLeftBumperPressed()) 
          {
            teleopMode = 1;
          } 
          else if (ctrOperator->GetRightBumperPressed()) {
            teleopMode = 2;
          }

          if (teleopMode == 0) 
          {
            innerPID.SetReference(inner_enc.GetPosition(), rev::CANSparkMax::ControlType::kPosition);
            outterPID.SetReference(outter_enc.GetPosition(), rev::CANSparkMax::ControlType::kPosition);
          }
          else if (teleopMode == 1) {
            

            if (ctrOperator->GetYButtonPressed()) 
            {
              teleopInit = false;
              outterPID.SetReference(-17.0f, rev::CANSparkMax::ControlType::kPosition);
            } 
            else if (ctrOperator->GetYButtonReleased()) 
            {
              innerPID.SetReference(-130.0f, rev::CANSparkMax::ControlType::kPosition);
              setManualXY = true;
              setLLXY = true;

            } 
            else if (ctrOperator->GetXButtonPressed()) 
            {
              outterPID.SetReference(-17.0f, rev::CANSparkMax::ControlType::kPosition);
              innerPID.SetReference(0, rev::CANSparkMax::ControlType::kPosition);
            }
            else if (ctrOperator->GetXButtonReleased()) 
            {
             //inner->Set(0.1);
              outterPID.SetReference(0, rev::CANSparkMax::ControlType::kPosition);
              
            }


          } 
          else if (teleopMode == 2) 
          {
              if (setManualXY) {
                MoveXY::Point xy = armCalc.command_to_xy(MoveXY::ArmAngles {inner_enc.GetPosition(), outter_enc.GetPosition()});
                currentPosition.armX = xy.x;
                currentPosition.armY = xy.y;
                setManualXY = false;
              }
              //movetoXY(innerSize, outterSize, true);
              MoveXY::Point xy = armCalc.command_to_xy(MoveXY::ArmAngles {inner_enc.GetPosition(), outter_enc.GetPosition()});
              currentPosition.armX = xy.x;
              currentPosition.armY = xy.y;

              if (ctrOperator->GetXButton()) 
              {
                if (ctrOperator->GetLeftTriggerAxis() > 0.5) 
                {
                  innerPID.SetReference(20, rev::CANSparkMax::ControlType::kPosition);
                  outterPID.SetReference(10, rev::CANSparkMax::ControlType::kPosition);
                } 
                else if (ctrOperator->GetRightTriggerAxis() >  0.5) 
                {
                  innerPID.SetReference(20, rev::CANSparkMax::ControlType::kPosition);
                  outterPID.SetReference(10, rev::CANSparkMax::ControlType::kPosition);
                }
              } 
              else if (ctrOperator->GetYButton()) 
              {
                if (ctrOperator->GetLeftTriggerAxis() > 0.5) 
                {
                  movetoPole(Limelight::topLeftPole);
                } 
                else if (ctrOperator->GetRightTriggerAxis() >  0.5) 
                {
                  movetoPole(Limelight::topRightPole);
                }
              } 
              else if (ctrOperator->GetAButton()) 
              {
                if (ctrOperator->GetLeftTriggerAxis() > 0.5) 
                {
                  movetoPole(Limelight::bottomLeftPole);
                } 
                else if (ctrOperator->GetRightTriggerAxis() >  0.5) 
                {
                  movetoPole(Limelight::bottomRightPole);
                }
              } 
              else 
              {
                double leftX = deadZoneCtr(ctrOperator->GetLeftX());
                double leftY = deadZoneCtr(ctrOperator->GetLeftY());
                jstickArmMovement(leftX, -leftY);

                
              }
          }
          else
          {
            
            
          }
        }

        if(state == 'a' ) {
          if(autoStart)
          {
            
            grabber->grabberMotor->Set(0.3);
            stow();
            
            //Arm Auto Init
            inner_enc.SetPosition(0);
            outter_enc.SetPosition(0);
            grabber->openAuto();

            //Stow
            //stow();
           // state = 'd';
            isFinished = true;

            autoStart = false;
            //grabber->openAuto();


            // stow();
            // inner_enc.SetPosition(stowInner);
            // outter_enc.SetPosition(stowOutter);
            // ShuffleUI::MakeWidget("InnerAngle", tab, clampAngle(inner_enc.GetPosition()));
            // ShuffleUI::MakeWidget("OutterAngle", tab, clampAngle(outter_enc.GetPosition()));
            
            // //moveProfiled(inner_enc.GetPosition(), outter_enc.GetPosition() - 20);
            // //movetoXY(0, 0, false);
            // ShuffleUI::MakeWidget("DONE", tab, 1);
            // isZero = true;
          }
          teleopInit = true;
          
        }
        if (state == 'd') {
          // isZero = false;
          // teleopInit = true;
          inner->Set(0);
          outter->Set(0);
          ShuffleUI::MakeWidget("disabled", tab, 0);
        }

        std::this_thread::sleep_until(nextRun);
        
    }
}

