#include <cmath>
#include <vector>
#include <iostream>
#include <tuple>
#include "ScaraArmModule.h"

void ScaraArmModule::ArmInit() {
  inner_enc.SetPositionConversionFactor(innerConv);
  outter_enc.SetPositionConversionFactor(outterConv);

  inner->SetInverted(true);
  outter->SetInverted(true);

  innerPID.SetOutputRange(-0.1, 0.1);
  outterPID.SetOutputRange(-0.1, 0.1);

  inner_enc.SetPosition(0);
  outter_enc.SetPosition(0);
  
  innerPID.SetP(0.1);
  innerPID.SetI(0.0);
  innerPID.SetD(0.0);
  outterPID.SetP(0.1);
  outterPID.SetI(0.0);
  outterPID.SetD(0.0);

  outter->SetSmartCurrentLimit(10);
  inner->SetSmartCurrentLimit(20);
}

std::vector<ScaraArmModule::armPos> ScaraArmModule::XY_to_Arm(double x, double y, double length1, double length2) {
  Circle c1(length2, x, y);
  Circle c2(length1, 0, 0);
  Circle::Point2d i1, i2;

  size_t i_points = c1.intersect(c2, i1, i2);
  if(i_points == 0) {
    std::vector<armPos> out { };
    return out;
  }

  std::vector<armPos> output = { };
  double theta1 = (atan2(i1.y(), i1.x())) * 180 / 3.141592;
  Circle::Point2d ab, ac;
  ab.X = i1.x() - 0;
  ab.Y = i1.y() - 0;
  ac.X = i1.x() - x;
  ac.Y = i1.y() - y;
  double angba = atan2(ab.Y, ab.X);
  double angbc = atan2(ac.Y, ac.X);
  double rslt = angba - angbc;
  double rs = (rslt * 180) / 3.141592;
  double theta2 = rs;
  armPos first_intersection;
  first_intersection.inner_angle = theta1;
  first_intersection.outter_angle = 180 - theta2;
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
    double rs = (rslt * 180) / 3.141592;
    double theta2 = rs;
    armPos second_intersection;
    second_intersection.inner_angle = theta1;
    second_intersection.outter_angle = 180 - theta2;
    output.push_back(second_intersection);
  }
  
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

  double current_outer = outter_enc.GetPosition();
  double current_inner = inner_enc.GetPosition();

  std::vector<armPos> angles = XY_to_Arm(x, y, innerSize, outterSize);
  std::cout << angles.at(0).inner_angle;
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



  if (angles.size() == 2) {

    frc::SmartDashboard::PutNumber("InnerCalc2", angles.at(1).inner_angle);
    frc::SmartDashboard::PutNumber("OutterCalc2", angles.at(1).outter_angle);

    /*Decision Code*/
    if (fabs(angles.at(0).outter_angle - current_outer) > fabs(angles.at(1).outter_angle - current_outer)) {
      innerPID.SetReference(angles.at(1).inner_angle, rev::CANSparkMax::ControlType::kPosition);
      outterPID.SetReference(angles.at(1).outter_angle, rev::CANSparkMax::ControlType::kPosition);
    } else {
      innerPID.SetReference(angles.at(0).inner_angle, rev::CANSparkMax::ControlType::kPosition);
      outterPID.SetReference(angles.at(0).outter_angle, rev::CANSparkMax::ControlType::kPosition);
    }
  } else {
    frc::SmartDashboard::PutNumber("InnerCalc2", -1);
    frc::SmartDashboard::PutNumber("OutterCalc2", -1);

    frc::SmartDashboard::PutBoolean("SecondSolution?", false);

    innerPID.SetReference(angles.at(0).inner_angle, rev::CANSparkMax::ControlType::kPosition);
    outterPID.SetReference(angles.at(0).outter_angle, rev::CANSparkMax::ControlType::kPosition);
  }


}

void ScaraArmModule::moveOnPath(double radius) {
  double start_inner = inner_enc.GetPosition();
  double start_outter = inner_enc.GetPosition();
  double current_inner = start_inner;
  double current_outter = start_outter;

  



  //Determine side of approach





  while (current_inner) {

  }
  /*
  Pseudocode
  1. Move to start of arc
  2. Generate points
  3. Nitish work on rest
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

  /*
  
  if (POV == -1) {
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
  */



    



  

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
