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
  inner->SetSmartCurrentLimit(10);
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
    return;
  }



  if (angles.size() == 2) {
    //Compute faster one here
    frc::SmartDashboard::PutNumber("InnerCalc2", angles.at(1).inner_angle);
    frc::SmartDashboard::PutNumber("OutterCalc2", angles.at(1).outter_angle);
    frc::SmartDashboard::PutBoolean("SecondSolution?", true);
    innerPID.SetReference(angles.at(1).inner_angle, rev::CANSparkMax::ControlType::kPosition);
    outterPID.SetReference(angles.at(1).outter_angle, rev::CANSparkMax::ControlType::kPosition);
   
  } else {
    frc::SmartDashboard::PutNumber("InnerCalc2", -1);
    frc::SmartDashboard::PutNumber("OutterCalc2", -1);
    frc::SmartDashboard::PutBoolean("SecondSolution?", false);
    innerPID.SetReference(angles.at(0).inner_angle, rev::CANSparkMax::ControlType::kPosition);
    outterPID.SetReference(angles.at(0).outter_angle, rev::CANSparkMax::ControlType::kPosition);
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
