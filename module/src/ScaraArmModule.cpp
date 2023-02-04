#include <cmath>
#include <vector>
#include <iostream>
#include <tuple>
#include "ScaraArmModule.h"

void ScaraArmModule::ArmInit() {
  inner_enc.SetPositionConversionFactor(innerConv);
  outter_enc.SetPositionConversionFactor(outterConv);
 // inner->SetInverted(true);
  //outterPID.SetOutputRange(-0.25, 0.25);
  //outter->SetInverted(true);
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



class Point2d : public ScaraArmModule {
    public:
    Point2d() {}
    Point2d(double x, double y)
        : X(x), Y(y) {}
     
    double x() const { return X; }
    double y() const { return Y; }
     
    /**
     * Returns the norm of this vector.
     * @return the norm
    */
    double norm() const {
        return sqrt( X * X + Y * Y );
    }
     
    void setCoords(double x, double y) {
        X = x; Y = y;
    }
     
    // Print point
    friend std::ostream& operator << ( std::ostream& s, const Point2d& p )  {
      s << p.x() << " " << p.y();
      return s;
    }
    double X;
    double Y;
};



class Circle : public ScaraArmModule {
public:
    /**
     * @param R - radius
     * @param C - center
     */
    Circle(double R, Point2d& C) 
        : r(R), c(C) {}
         
    /**
     * @param R - radius
     * @param X - center's x coordinate
     * @param Y - center's y coordinate
     */
    Circle(double R, double X, double Y) 
        : r(R), c(X, Y) {}    
     
    Point2d getC() const { return c; }
    double getR() const { return r; }
     
    size_t intersect(const Circle& C2, Point2d& i1, Point2d& i2) {
        // distance between the centers
        double d = Point2d(c.x() - C2.c.x(), 
                c.y() - C2.c.y()).norm();
         
        // find number of solutions
        if(d > r + C2.r) // circles are too far apart, no solution(s)
        {
            std::cout << "Circles are too far apart\n";
            return 0;
        }
        else if(d == 0 && r == C2.r) // circles coincide
        {
            std::cout << "Circles coincide\n";
            return 0;
        }
        // one circle contains the other
        else if(d + min(r, C2.r) < max(r, C2.r))
        {
            std::cout << "One circle contains the other\n";
            return 0;
        }
        else
        {
            double a = (r*r - C2.r*C2.r + d*d)/ (2.0*d);
            double h = sqrt(r*r - a*a);
             
            // find p2
            Point2d p2( c.x() + (a * (C2.c.x() - c.x())) / d,
                    c.y() + (a * (C2.c.y() - c.y())) / d);
             
            // find intersection points p3
            i1.setCoords( p2.x() + (h * (C2.c.y() - c.y())/ d),
                    p2.y() - (h * (C2.c.x() - c.x())/ d)
            );
            i2.setCoords( p2.x() - (h * (C2.c.y() - c.y())/ d),
                    p2.y() + (h * (C2.c.x() - c.x())/ d)
            );
             
            if(d == r + C2.r)
                return 1;
            return 2;
        }
    }
     
    // Print circle
    friend std::ostream& operator << ( std::ostream& s, const Circle& C )  {
      s << "Center: " << C.getC() << ", r = " << C.getR();
      return s;
    }
private:
    // radius
    double r;
    // center
    Point2d c;
     
};

std::vector<double> ScaraArmModule::XY_to_Arm(double x, double y, double length1, double length2) {


    Circle c1(length2, x, y);
    Circle c2(length1, 0, 0);
    Point2d i1, i2;
     
    //std::cout << c1 << "\n" << c2 << "\n";
    // intersections point(s)
    size_t i_points = c1.intersect(c2, i1, i2);
     
    //std::cout << "Intersection point(s)\n";
    if(i_points == 0) {
      std::vector<double> null {};
      return null;
    }
    
    std::vector<double> output = { };

    
    //Compute angles
    double theta1 = (atan2(i1.y(), i1.x())) * 180 / 3.141592;
    Point2d ab, ac;
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
    output.push_back(theta1);
    output.push_back(theta2);


    if (i_points == 2) {
      double theta1 = (atan2(i2.y(), i2.x())) * 180 / 3.141592;
      Point2d ab, ac;
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
      output.push_back(theta1);
      output.push_back(theta2);
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
  std::vector<double> angles = XY_to_Arm(x, y, innerSize, outterSize);
  double currPos = clampAngle(inner_enc.GetPosition());
  for (int i = 0; i < angles.size(); i++) {
    angles.at(i) = clampAngle(angles.at(i));
    std::cout << angles.at(i);
  }
  
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
