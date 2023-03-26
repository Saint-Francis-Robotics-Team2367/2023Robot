#include <rev/CANSparkMax.h>
#include "Macros.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include <thread>
#include <chrono>
#include<mutex>
#include<frc/XboxController.h>
#include <atomic>
#include <frc/Timer.h>
#define triggerDeadband 0.1
#define PI 3.141592654 // 3.1415926

#pragma once

class ElevatorModule {

    frc::XboxController* ctr;
    frc::XboxController* ctrOperator;
    int run_counter = 0;
    double slowCoefficient = 5;
    public:
    //Conversion Factors:
    double pitch_diameter = 1.273;
    double pitch_circum = pitch_diameter * PI;
    double conversion_factor = pitch_circum / (5); //Ratio 30:1
    std::thread elevatorThread;
    char state = 't';
    double stopAuto = false;
    bool test = true;

    bool isRunningAuto = false;
    bool isFinished = false;
    double autoAmount = 0;
    void autoSet(double setpoint);


    void zeroAtTop();

    std::string tab = "Elevator"; // for MakeWidget() calls

    enum poleIDs {
        bottomRightPole = 0,
        bottomLeftPole = 1,
        topRightPole = 2,
        topLeftPole = 3
    };

    double maxVelocity = 25047;
    double maxAcc = 5039;
    //Mech/Electronics Setup
    //int m_ID = 10;
    //int m_ID = 15;
    

    rev::CANSparkMax* elevatorMotor = new rev::CANSparkMax(elevatorID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder enc = elevatorMotor->GetEncoder();
    rev::SparkMaxPIDController elevatorPID = elevatorMotor->GetPIDController();
    rev::SparkMaxLimitSwitch topSwitch = elevatorMotor->GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen);

    double height = 0; //starting the elevator at 0 (no absolute encoder)

    //constants
    double kElevatorMinHeight = 0.0;
    double kElevatorMaxHeight = 34.75;
    double kLowScoreHeight = 25;
    double kHighScoreHeight = 26.75;
    double kHighIntakeHeight = 26.375;
    double kLowestHeight = 0.0;

    bool currentlyMoving = false;

    //no feedforward in this case...
    double pDown = 0.20;
    double dDown = 0;

    double pUp = 0.1; 
    double dUp = 0.0;

    bool oneRun = true;


    ElevatorModule(frc::XboxController* controller, frc::XboxController* controllerOperator);
    void Init();
    void TeleopPeriodic(double Linput, double Rinput);
    void AutoPeriodic();
    double getPos();
    void resetPos();
    double manualMove(double Linput);
    void setPos(double setpoint);
    bool setPos(double setpoint, bool isMotionProfiled);
    double getHeight();
    void run();
    void runInit();
    private:
};