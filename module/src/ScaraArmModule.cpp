#include <cmath>
#include <vector>
#include <iostream>
#include <tuple>
#include "ScaraArmModule.h"

ScaraArmModule::ScaraArmModule(frc::XboxController* controller) {
  ctr = controller;
  scaraArmThread = std::thread(&ScaraArmModule::run, this); //initializing thread so can detach in robot init
  inner->SetSmartCurrentLimit(20);
  outter->SetSmartCurrentLimit(20);
}

void ScaraArmModule::ArmInit() {
  inner_enc.SetPositionConversionFactor(innerConv);
  outter_enc.SetPositionConversionFactor(outterConv);
  inner->SetInverted(true);
  //outterPID.SetOutputRange(-0.25, 0.25);
  outter->SetInverted(true);
  inner_enc.SetPosition(0);
  outter_enc.SetPosition(0);
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
  outterPID.SetOutputRange(-0.1, 0.1);

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

}

void ScaraArmModule::run(){
   runInit();
    while(true) {
        auto nextRun = std::chrono::steady_clock::now() + std::chrono::milliseconds(5); //change milliseconds at telop
        frc::SmartDashboard::PutBoolean("scara arm module", true);

        if(state = 't') {
          frc::SmartDashboard::PutNumber("left y scara arm", ctr->GetLeftY());
          inner->Set(ctr->GetLeftY() / 5);
          frc::SmartDashboard::PutNumber("right y", ctr->GetRightY());
          outter->Set(ctr->GetRightY()/ 5);
        }

        if(state = 'a') {
          
        }


        std::this_thread::sleep_until(nextRun);
        
    }
}
