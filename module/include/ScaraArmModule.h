#include <cmath>
#include <cstdlib>
#include <vector>
#include <iostream>
#include <tuple>

#include <rev/CANSparkMax.h>
#include <frc/SmartDashboard/SmartDashboard.h>
#include <thread>
#include <chrono>
#include<mutex>
#include <atomic>
#include "Macros.h"
#include<frc/XboxController.h>

#pragma once

#ifndef max
  #define max(a,b) (((a) > (b)) ? (a) : (b))
#endif
#ifndef min
  #define min(a,b) (((a) < (b)) ? (a) : (b))
#endif
 



class ScaraArmModule {
    public:

    const double startX = 24.625;
    const double startY = 0;

    // const int innerID = 10;
    // const int outterID = 14;

    const double innerConv = 6.15234; //1/(4096/70/360); 
    const double outterConv = 4.394; //1/(4096/70/360); //50 now

    const double innerSize = 24.625;
    const double outterSize = 24.500;
    

    std::thread scaraArmThread;
    double stopAuto = false;

    frc::XboxController* ctr;
    ScaraArmModule(frc::XboxController* controller);
    char state = 't';
    bool test = true;
    void ArmInit();
    void run();
    void runInit();
    
    struct armPos {
        double inner_angle;
        double outter_angle;
        double armX;
        double armY;
        bool PossibleReach = true;
    };

    std::vector<armPos> XY_to_Arm(double x, double y, double length1, double length2);

    std::vector<double> Angles_to_XY(double inner, double outter);

    double clampAngle(double inp);

    void movetoXY(double x, double y);


    armPos currentPosition = {0.0, 0.0, startX, startY};
    armPos calculatedPosition = {0.0, 0.0, startX, startY};
    

    rev::CANSparkMax* inner = new rev::CANSparkMax(scaraArmInner, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder inner_enc = inner->GetEncoder();
    rev::SparkMaxPIDController innerPID = inner->GetPIDController();
    
    rev::CANSparkMax* outter = new rev::CANSparkMax(scaraArmOutter, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder outter_enc = outter->GetEncoder();
    rev::SparkMaxPIDController outterPID = outter->GetPIDController();



class Circle {
public:
class Point2d {
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



};


