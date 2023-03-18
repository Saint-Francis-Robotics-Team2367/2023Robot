#include <rev/CANSparkMax.h>
#include "Macros.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include <chrono>
#include<frc/XboxController.h>
#include <thread>

class CubeIntakeModule {
    public: 
        CubeIntakeModule(frc::XboxController* controller, frc::XboxController* controllerOperator);
        void toggleIntake();
        void shootToGoal();
        std::thread cubeIntakeThread;
        char state = 't';
        void run(); //could be private
        void runInit();
    private:
        bool setMotorPIDF(rev::CANSparkMax* motor, double P, double I, double D, double F);
        double kP = 0.00008;
        double kI = 0.0; 
        double Iz = 0.0;
        double kD = 0.0;
        double FF = 0.00017; //linear regression to find FF //this only works for neo550?
        double maxShooterOutput = 0.1;
        double minShooterOutput = -1.0;

        bool intakeDown = false;

        frc::XboxController* ctr;
        frc::XboxController* ctrOperator;

        rev::CANSparkMax * leftStar = new rev::CANSparkMax(leftStarID, rev::CANSparkMax::MotorType::kBrushed);
        rev::CANSparkMax * rightStar = new rev::CANSparkMax(rightStarID, rev::CANSparkMax::MotorType::kBrushed);
        rev::CANSparkMax * intakeRoller = new rev::CANSparkMax(intakeRollerID, rev::CANSparkMax::MotorType::kBrushless);  

        rev::SparkMaxRelativeEncoder intakeRollerEncoder = leftStar->GetEncoder();
        rev::SparkMaxPIDController intakeRollerPID = leftStar->GetPIDController();
        rev::SparkMaxRelativeEncoder rotaryEncoder = leftStar->GetEncoder();
        rev::SparkMaxPIDController rotaryrPID = leftStar->GetPIDController();

        
    
};