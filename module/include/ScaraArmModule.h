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
#include <Grabber.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include "Limelight.h"
#include <rev/SparkMaxLimitSwitch.h>
#include <frc/DigitalInput.h>
#include "moveXY.h"


#pragma once

#ifndef max
  #define max(a,b) (((a) > (b)) ? (a) : (b))
#endif
#ifndef min
  #define min(a,b) (((a) < (b)) ? (a) : (b))
#endif
 
#define EPS 1.0E-3


class ScaraArmModule {
    public:

    // const double startInner = 146 + 180;
    //startOutter is 141.189407
    //startInner is 229.16436
    // const double startOutter = 150;
    const double startInner = 141.189407;
    const double startOutter = -1 * (360 - 229.16436);

    const double stowOutter = 141.189407;
    const double stowInner = -1 * (360 - 229.16436);

    const double startX = 24.625;
    const double startY = 0;

    // const int innerID = 10;
    // const int outterID = 14;

    // const double innerConv = 6.15234; //1/(4096/70/360); 
    // const double outterConv = 4.394; //1/(4096/70/360); //50 now
    //36 / 7; 
    //1/(4096/70/360); 

    const double innerConv = 3.6; 
    const double outterConv = 36 / 7; //1

    const double innerSize = 23.75;
    const double outterSize = 31.5;

    std::string tab = "Scara Arm"; // for MakeWidget() calls
    bool scoreMenuCreated = false; // for ShuffleboardScorer()

    Grabber* grabber = new Grabber();

    MoveXY armCalc = MoveXY(310, stowOutter + 180, innerSize, outterSize);

    void movetoPole(Limelight::poleIDs poleID);


    //frc::ShuffleboardTab& armTab = frc::Shuffleboard::GetTab("Arm");


    
    //Shuffleboard Tabs:
    //NetworkTableEntry myBoolean = Shuffleboard.getTab("Example Tab").add("My Boolean", false)



    // Can also use a different widget type:
    // .WithWidget(&frc::BuiltInWidgets::kSplitButtonChooser);

    Limelight ll;



    // frc::DigitalInput InnerLimitSwitch {0};
    // frc::DigitalInput OuterLimitSwitch {1};
    std::thread scaraArmThread;
    double stopAuto = false;
    frc::XboxController* ctr;
    frc::XboxController* ctrOperator;
    ScaraArmModule(frc::XboxController* controller, frc::XboxController* controllerOperator);
    char state = 'd';
    bool test = true;
    bool autoStart = false;
    bool isFinished = false;
    //bool isManualMove = false;
    void ArmInit();
    void run();
    void runInit();
    void stow();
    double deadZoneCtr(double inp);
    void jstickArmMovement(double jstickX, double jstickY);

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


    

    armPos currentPosition = {startInner, startOutter, startX, startY};
    armPos calculatedPosition = {0.0, 0.0, startX, startY};

    //Unstow Path Points, WE ARE NOT USING ANGLES PLS DONT HATE ME
    
    armPos backRight = {-1, -1, 0, 0};
    armPos backLeft = {-1, -1, 0, 0};
    armPos frontLeft = {-1, -1, 0, 0};
    std::vector<armPos> reverseStowPositions = {backRight, backLeft, frontLeft};
    

    rev::CANSparkMax* inner = new rev::CANSparkMax(scaraArmInner, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder inner_enc = inner->GetEncoder();
    rev::SparkMaxPIDController innerPID = inner->GetPIDController();
    rev::SparkMaxLimitSwitch InnerLimitSwitch = inner->GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyClosed);
    
    rev::CANSparkMax* outter = new rev::CANSparkMax(scaraArmOutter, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder outter_enc = outter->GetEncoder();
    rev::SparkMaxPIDController outterPID = outter->GetPIDController();
    rev::SparkMaxLimitSwitch OutterLimitSwitch = outter->GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyClosed);




};


