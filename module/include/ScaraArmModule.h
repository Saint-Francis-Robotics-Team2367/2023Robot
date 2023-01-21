#include <cmath>
#include <cstdlib>
#include <vector>
#include <iostream>
#include <tuple>

#include <rev/CANSparkMax.h>
#include <frc/SmartDashboard/SmartDashboard.h>

#ifndef max
  #define max(a,b) (((a) > (b)) ? (a) : (b))
#endif
#ifndef min
  #define min(a,b) (((a) < (b)) ? (a) : (b))
#endif
 



class ScaraArmModule {
    public:

    int innerID = 0;
    int outterID = 1;

    const int innerConv = 1;
    const int outterConv = 1;

    const int innerSize = 30;
    const int outterSize = 30;
    
    void ScaraArmModule::ArmInit();

    std::vector<double> XY_to_Arm(double x, double y, double length1, double length2);

    std::vector<double> Angles_to_XY(double inner, double outter);

    void movetoXY(double x, double y);

    rev::CANSparkMax* inner = new rev::CANSparkMax(innerID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder inner_enc = inner->GetEncoder();
    rev::SparkMaxPIDController innerPID = inner->GetPIDController();
    
    rev::CANSparkMax* outter = new rev::CANSparkMax(outterID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder outter_enc = outter->GetEncoder();
    rev::SparkMaxPIDController outterPID = outter->GetPIDController();



};


