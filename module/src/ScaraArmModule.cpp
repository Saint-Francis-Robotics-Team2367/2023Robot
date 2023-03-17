#include <cmath>
#include <vector>
#include <iostream>
#include <tuple>
#include "ScaraArmModule.h"
#include <frc/Timer.h>

ScaraArmModule::ScaraArmModule(frc::XboxController* controller) {
  ctr = controller;
  scaraArmThread = std::thread(&ScaraArmModule::run, this); //initializing thread so can detach in robot init
  inner->SetSmartCurrentLimit(20);
  outter->SetSmartCurrentLimit(20);
}

void ScaraArmModule::ArmInit() {
  inner_enc.SetPositionConversionFactor(innerConv);
  outter_enc.SetPositionConversionFactor(outterConv);
  inner->SetInverted(false);
  //outterPID.SetOutputRange(-0.25, 0.25);
  outter->SetInverted(false);
  inner_enc.SetPosition(0);
  outter_enc.SetPosition(0);
  inner->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  inner->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  inner_enc.SetPosition(startInner);
  outter_enc.SetPosition(startOutter);
  innerPID.SetP(0.1);
  innerPID.SetI(0.0);
  innerPID.SetD(0.0);
  outterPID.SetP(0.1);
  outterPID.SetI(0.0);
  outterPID.SetD(0.0);

  outter->SetSmartCurrentLimit(10);
  inner->SetSmartCurrentLimit(10);
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









std::vector<ScaraArmModule::armPos> ScaraArmModule::XY_to_Arm(double x, double y, double length1, double length2) {
    Circle c1(length2, x, y);
    Circle c2(length1, 0, 0);
    Circle::Point2d i1, i2;
     
    //std::cout << c1 << "\n" << c2 << "\n";
    // intersections point(s)
    size_t i_points = c1.intersect(c2, i1, i2);
     
    //std::cout << "Intersection point(s)\n";
    if(i_points == 0) {
      /*
      std::vector<double> null {};
      frc::SmartDashboard::PutBoolean("0INTERSECTIONS", true);
      return null;
      */
     //armPos no_reach;
     //no_reach.PossibleReach = false;
     std::vector<armPos> out { };
     return out;
    }
    
    std::vector<armPos> output = { };

    
    //Compute angles
    double theta1 = (atan2(i1.y(), i1.x())) * 180 / 3.141592;
    Circle::Point2d ab, ac;
    ab.X = i1.x() - 0;
    ab.Y = i1.y() - 0;
    ac.X = i1.x() - x;
    ac.Y = i1.y() - y;
    double angba = atan2(ab.Y, ab.X);
    double angbc = atan2(ac.Y, ac.X);
    double rslt = angba - angbc;
    //std::cout << rslt;
    double rs = (rslt * 180) / 3.141592;
    //std::cout << rs;
    double theta2 = rs;
    armPos first_intersection;
    first_intersection.inner_angle = theta1;
    first_intersection.outter_angle = 180 - theta2;
    //output.push_back(theta1);
    //output.push_back(theta2);
    output.push_back(first_intersection);


    if (i_points == 2) {
      double theta1 = (atan2(i2.y(), i2.x())) * 180 / 3.141592;
      Circle::Point2d ab, ac;
      ab.X = i2.x() - 0;
      ab.Y = i2.y() - 0;
      ac.X = i2.x() - x;
      ac.Y = i2.y() - y;
      double angba = atan2(ab.Y, ab.X);
      double angbc = atan2(ac.Y, ac.X);
      double rslt = angba - angbc;
      //std::cout << rslt;
      double rs = (rslt * 180) / 3.141592;
      //std::cout << rs;
      double theta2 = rs;
      armPos second_intersection;
      second_intersection.inner_angle = theta1;
      second_intersection.outter_angle = 180 - theta2;
      //output.push_back(theta1);
      //output.push_back(theta2);
      output.push_back(second_intersection);
    }
    
    //std::cout << "Theta1: " << theta1 * 180/ M_PI << "Theta2: " << theta2;
    
    return output;

}

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

void ScaraArmModule::movetoXY(double x, double y) {
  // inner_enc.SetPosition(0); //ignore and have a global one so we bing chilling
  // outter_enc.SetPosition(0); //will need to change this system to clamp and add to stuff
  std::vector<armPos> angles = XY_to_Arm(x, y, innerSize, outterSize);
  double currPos = clampAngle(inner_enc.GetPosition());
  for (int i = 0; i < angles.size(); i++) {
    angles.at(i).inner_angle = clampAngle(angles.at(i).inner_angle);
    angles.at(i).outter_angle = clampAngle(angles.at(i).outter_angle);
  }
  frc::SmartDashboard::PutNumber("NumSol", angles.size());

  if (angles.size() > 0) {
    frc::SmartDashboard::PutNumber("InnerCalc1", angles.at(0).inner_angle);
    frc::SmartDashboard::PutNumber("OutterCalc1", angles.at(0).outter_angle);
  }

  /*
  if(angles.size() == 2) {
    // innerPID.SetReference(angles.at(0), rev::CANSparkMax::ControlType::kPosition);
    // outterPID.SetReference(angles.at(1), rev::CANSparkMax::ControlType::kPosition);
  }
  */

  innerPID.SetOutputRange(-0.1, 0.1);
  outterPID.SetOutputRange(-0.1, 0.1); //change this after motion profiling

  if (angles.size() == 2) {
    
    frc::SmartDashboard::PutNumber("InnerCalc2", angles.at(1).inner_angle);
    frc::SmartDashboard::PutNumber("OutterCalc2", angles.at(1).outter_angle);
    frc::SmartDashboard::PutBoolean("SecondSolution?", true);
    //add the chooser here to choose which one to do
    innerPID.SetReference(angles.at(1).inner_angle, rev::CANSparkMax::ControlType::kPosition);
    outterPID.SetReference(angles.at(1).outter_angle, rev::CANSparkMax::ControlType::kPosition);
   
  } else {
    frc::SmartDashboard::PutNumber("InnerCalc2", -1);
    frc::SmartDashboard::PutNumber("OutterCalc2", -1);
    frc::SmartDashboard::PutBoolean("SecondSolution?", false);

    innerPID.SetReference(angles.at(0).inner_angle, rev::CANSparkMax::ControlType::kPosition);
    outterPID.SetReference(angles.at(0).outter_angle, rev::CANSparkMax::ControlType::kPosition);
  }

 
 

  /*
  if (angles.size() == 2) {
    innerPID.SetReference(angles.at(0), rev::CANSparkMax::ControlType::kPosition);
    outterPID.SetReference(angles.at(1), rev::CANSparkMax::ControlType::kPosition);
  } else if (angles.size() == 4) {
    if ((currPos - angles[0]) > (currPos - angles[2])) {
      innerPID.SetReference(angles.at(2), rev::CANSparkMax::ControlType::kPosition);
      outterPID.SetReference(angles.at(3), rev::CANSparkMax::ControlType::kPosition);
    } else {
      innerPID.SetReference(angles.at(0), rev::CANSparkMax::ControlType::kPosition); 
      outterPID.SetReference(angles.at(1), rev::CANSparkMax::ControlType::kPosition);
    }
  } else {
    inner->Set(0);
    outter->Set(0);

  }
  */
}

void ScaraArmModule::moveProfiled(double angleInner, double angleOutter) {
    float timeElapsedInner, DistanceToDeccelerateInner, currentVelocityInner = 0.0; //currentPositionInner is the set point
    double currentPositionInner = inner_enc.GetPosition();; //current velocity is a class variable
    float prevTimeInner = frc::Timer::GetFPGATimestamp().value();

    float timeElapsedOutter, DistanceToDeccelerateOutter, currentVelocityOutter = 0.0; //currentPositionOutter is the set point
    double currentPositionOutter = outter_enc.GetPosition(); //current velocity is a class variable
    float prevTimeOutter = frc::Timer::GetFPGATimestamp().value();


    while(fabs(currentPositionOutter - angleOutter) > 0 && fabs(currentPositionInner - angleInner) > 0){

    if(fabs(currentPositionInner - angleInner) > 0) {
      if(stopAuto) {
          break;
        }
        timeElapsedInner = frc::Timer::GetFPGATimestamp().value() - prevTimeInner;
        DistanceToDeccelerateInner = (3 * currentVelocityInner * currentVelocityInner) / (2 * maxAcc); //change
        if (fabs(DistanceToDeccelerateInner) > fabs(angleInner - currentPositionInner)) {
          currentVelocityInner -= (maxAcc * timeElapsedInner);
        }
        else //increase velocity
        {
          currentVelocityInner += (maxAcc * timeElapsedInner);
          if (fabs(currentVelocityInner) > fabs(maxVelocity))
          {
            currentVelocityInner = maxVelocity;
          }
        }

        currentPositionInner += currentVelocityInner * timeElapsedInner;
        // if(fabs(currentPositionInner) > fabs(angleInner)) {
        //   currentPositionInner = angleInner;
        // }
        frc::SmartDashboard::PutNumber("angleInner", angleInner);
        if(angleInner < 0) {
            if(currentPositionInner  < angleInner) {   
                currentPositionInner = angleInner;
            }
        } else {
            if(currentPositionInner > angleInner) {
                currentPositionInner = angleInner;
            }
        }
        innerPID.SetReference(std::copysign(currentPositionInner, angleInner), rev::CANSparkMax::ControlType::kPosition); //angleInner uses encoder
        prevTimeInner = frc::Timer::GetFPGATimestamp().value();
        frc::SmartDashboard::PutNumber("prevTimeInner", prevTimeInner);
    }
  
  
	  if(fabs(currentPositionOutter - angleOutter) > 0) {
		        if(stopAuto) {
         			 break;
            }

        timeElapsedOutter = frc::Timer::GetFPGATimestamp().value() - prevTimeOutter;
        DistanceToDeccelerateOutter = (3 * currentVelocityOutter * currentVelocityOutter) / (2 * maxAcc); //change
        if (fabs(DistanceToDeccelerateOutter) > fabs(angleOutter - currentPositionOutter)) {
          currentVelocityOutter -= (maxAcc * timeElapsedOutter);
        }
        else //increase velocity
        {
          currentVelocityOutter += (maxAcc * timeElapsedOutter);
          if (fabs(currentVelocityOutter) > fabs(maxVelocity))
          {
            currentVelocityOutter = maxVelocity;
          }
        }

        currentPositionOutter += currentVelocityOutter * timeElapsedOutter;
        // if(fabs(currentPositionOutter) > fabs(angleOutter)) {
        //   currentPositionOutter = angleOutter;
        // }

        if(angleOutter < 0) {
            if(currentPositionOutter  < angleOutter) {   
                currentPositionOutter = angleOutter;
            }
        } else {
            if(currentPositionOutter > angleOutter) {
                currentPositionOutter = angleOutter;
            }
        }
        frc::SmartDashboard::PutNumber("angleOutter", angleOutter);
        outterPID.SetReference(std::copysign(currentPositionOutter, angleOutter), rev::CANSparkMax::ControlType::kPosition); //angleOutter uses encoder
        prevTimeOutter = frc::Timer::GetFPGATimestamp().value();
        frc::SmartDashboard::PutNumber("prevTimeOutter", prevTimeOutter);
    }
	}
}

void ScaraArmModule::TeleopInit() {
  currentPosition.inner_angle = inner_enc.GetPosition();
  currentPosition.outter_angle = outter_enc.GetPosition();
  std::vector<double> curr_xy = Angles_to_XY(inner_enc.GetPosition(), outter_enc.GetPosition());
  if (XYInRange(curr_xy.at(0), curr_xy.at(1))) {
    currentPosition.armX = curr_xy.at(0);
    currentPosition.armY = curr_xy.at(1);
  } else {
    frc::SmartDashboard::PutBoolean("Init Failed", true);
  }
  

}

void ScaraArmModule::TeleopControl(int POV) {// Angle from 0 - 315
  
  /*
  if (fabs(in) > 0.1) {
    inner->Set(in / 6);
  } else {
    inner->Set(0);
  }
  if (fabs(out) > 0.1) {
    outter->Set(out / 6);
  } else {
    outter->Set(0);
  }
  */
  //int dPadInput = POV;

  
  frc::SmartDashboard::PutNumber("POV", POV);
  if (POV == -1) {
    // outter->Set(0);
    // inner->Set(0);
    
    frc::SmartDashboard::PutNumber("movetoX", currentPosition.armX);
    frc::SmartDashboard::PutNumber("movetoY", currentPosition.armY);
    if (XYInRange(currentPosition.armX, currentPosition.armY)) {
      movetoXY(currentPosition.armX, currentPosition.armY);
      frc::SmartDashboard::PutBoolean("Invalid Point", false);
    } else {
      frc::SmartDashboard::PutBoolean("Invalid Point", true);
    }
    
    
  } else {
    double x_increment = cos(POV * M_PI / 180);
    double y_increment = sin(POV * M_PI / 180);
    frc::SmartDashboard::PutNumber("movetoX", currentPosition.armX);
    frc::SmartDashboard::PutNumber("movetoY", currentPosition.armY);
    frc::SmartDashboard::PutNumber("X_inc", x_increment);
    frc::SmartDashboard::PutNumber("Y_inc", y_increment);
  
    currentPosition.armX += x_increment;
    currentPosition.armY += y_increment;
    if (XYInRange(currentPosition.armX, currentPosition.armY)) {
      movetoXY(currentPosition.armX, currentPosition.armY);
      frc::SmartDashboard::PutBoolean("Invalid Point", false);
    } else {
      frc::SmartDashboard::PutBoolean("Invalid Point", true);
    }
   
  }
}

double ScaraArmModule::clampAngle(double inp) {
double out;
  out = fmod(inp, 360);
  if (fabs(out) > 180) {
    if (out > 0) {
      return out - 360;
    } else {
      return 360 + out;
    }

  } else {
    return out;
  }
}



void ScaraArmModule::runInit() {
  ArmInit();
  TeleopInit();
  grabber->Init();
  grabber->grabberMotor->SetSmartCurrentLimit(10);
}

void ScaraArmModule::run(){
   runInit();
   int counter = 0;
    while(true) {
        auto nextRun = std::chrono::steady_clock::now() + std::chrono::milliseconds(5); //change milliseconds at telop
        frc::SmartDashboard::PutBoolean("scara arm module", true);
        

        if(state == 't') {
          //TeleopControl(ctr->GetPOV());
            frc::SmartDashboard::PutNumber("left y scara arm", ctr->GetLeftY());
            inner->Set((ctr->GetRightTriggerAxis() - ctr->GetLeftTriggerAxis()) / 2);
            frc::SmartDashboard::PutNumber("right y", ctr->GetRightY());
            int pov = ctr->GetPOV();
            if (pov == 90) {
              outter->Set(0.2);
            } else if (pov == 270) {
              outter->Set(-0.2);
            } else {
              outter->Set(0);
            }

          //grabber->set(ctr->GetLeftTriggerAxis() - ctr->GetRightTriggerAxis());
          
          if(ctr->GetBButtonPressed()) {
            counter += 1;
            counter = counter % 3;
          }

          if(counter == 0) {
            grabber->grabberMotor->StopMotor();
          } else if (counter == 1) {
            grabber->set(1);
          } else {
            grabber->set(-1.0);
          }
          frc::SmartDashboard::PutNumber("InnerAngle", inner_enc.GetPosition());
          frc::SmartDashboard::PutNumber("OutterAngle", outter_enc.GetPosition());
        }

        if(state == 'a') {
          
        }

        std::this_thread::sleep_until(nextRun);
        
    }
}

bool ScaraArmModule::XYInRange(double x, double y) {
  double circle_x = 0;
  double circle_y = 0;
  double rad = outterSize + innerSize;
  if ((x - circle_x) * (x - circle_x) +
        (y - circle_y) * (y - circle_y) <= rad * rad)
        return true;
    else
        return false;
  //Check if x,y is valid 

}

void ScaraArmModule::checkArmBounds(double outter_pos, double outter_neg, double inner_pos, double inner_neg) {
  double out_pos  = outter_enc.GetPosition();
  double in_pos = inner_enc.GetPosition();
  frc::SmartDashboard::PutBoolean("Arm Bounded?", true);

  if (out_pos > outter_pos || out_pos < outter_neg) {
    outter->Set(0);
    frc::SmartDashboard::PutBoolean("Arm Bounded?", true);

  } else {
    frc::SmartDashboard::PutBoolean("Arm Bounded?", false);
  }
  if (in_pos > inner_pos || in_pos < inner_neg) {
    inner->Set(0);
    frc::SmartDashboard::PutBoolean("Arm Bounded?", true);
  } else {
    frc::SmartDashboard::PutBoolean("Arm Bounded?", false);
  }
}
