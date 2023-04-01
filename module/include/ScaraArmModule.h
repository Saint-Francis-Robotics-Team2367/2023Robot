#include <cmath>
#include <cstdlib>
#include <vector>
#include <iostream>
#include <tuple>

#include "Macros.h"
#include "Limelight.h"
//#include "Grabber.h"
#include "MoveXY.h"


#include <rev/CANSparkMax.h>
#include<frc/XboxController.h>
#include <rev/SparkMaxLimitSwitch.h>
#include <frc/DigitalInput.h>

#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/DriverStation.h>

#include <thread>
#include <chrono>
#include<mutex>
#include <atomic>


#ifndef max
  #define max(a,b) (((a) > (b)) ? (a) : (b))
#endif
#ifndef min
  #define min(a,b) (((a) < (b)) ? (a) : (b))
#endif
 
#define EPS 1.0E-3


class ScaraArmModule {
    public:

    const double stowOutter = 141.189407;
    const double stowInner = 130.83564;

    const double innerConv = 3.6;  //1/(4096/100/360)
    const double outterConv = 36 / 7; //1/(4096/70/360)

    const double innerSize = 23.75;
    const double outterSize = 31.5;
    
    double maxVelocity = 130;
    double maxAcc = 68;

    struct armProfile{
        float timeElapsed, DistanceToDeccelerate, currentVelocity = 0.0; //currentPositionInner is the set point
        double currentPosition; //current velocity is a class variable
        float prevTime;
        bool positive;
        bool isProfiling = false;
        bool firstRun = true;
        double startPosition;
    };

    enum motorMappings {
        innerMotor = 0,
        outterMotor = 1
    };

    armProfile innerProfile;
    armProfile outterProfile;
        

    struct armPos {
        double inner_angle;
        double outter_angle;
        double armX;
        double armY;
        bool PossibleReach = true;
    };

    struct PointXY {
        double x;
        double y;
    };

    //Grabber* grabber = new Grabber();
    Limelight ll;
    frc::XboxController* ctr;
    frc::XboxController* ctrOperator;
    MoveXY armCalc = MoveXY((360 - stowInner) + 90, 180 + stowOutter, innerSize, outterSize);

    std::thread scaraArmThread;
    double stopAuto = false;
    std::string tab = "Scara Arm"; // for MakeWidget() calls
    bool scoreMenuCreated = false; // for ShuffleboardScorer()

    char state = 'd';
    bool test = true;
    bool autoStart = false;
    bool isFinished = false;
    bool isStowing = false;
    armPos currentPosition;

    ScaraArmModule(frc::XboxController* controller, frc::XboxController* controllerOperator);
    void run();
    void runInit();
    double deadZoneCtr(double inp);
    void stow(double innerSet, double outterSet, double outterSlowSet);
    bool moveProfiled(double setpoint, motorMappings motor);
    PointXY getPoleXY(Limelight::poleIDs poleID);
    void jstickArmMovement(double jstickX, double jstickY);
    bool XYInRange(double x, double y);


    rev::CANSparkMax* inner = new rev::CANSparkMax(scaraArmInner, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder inner_enc = inner->GetEncoder();
    rev::SparkMaxPIDController innerPID = inner->GetPIDController();
    rev::SparkMaxLimitSwitch InnerLimitSwitch = inner->GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyClosed);
    
    rev::CANSparkMax* outter = new rev::CANSparkMax(scaraArmOutter, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder outter_enc = outter->GetEncoder();
    rev::SparkMaxPIDController outterPID = outter->GetPIDController();
    rev::SparkMaxLimitSwitch OutterLimitSwitch = outter->GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyClosed);





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


