#include <rev/CANSparkMax.h>
#pragma once


#define motorInitMaxCurrent 40 // The initial max current setting
#define motorInitRatedCurrent 30 // The inital rated current settings
#define motorInitLimitCycles 200 // The inital number of allowed ms at peak current
#define lInvert false // Inversion setings for sides (invert this if opposite side)
#define rInvert true 


//for drive base, would change based on which motor it is though

//default PID's for drive
#define kP 5e-5
#define kI 1e-6
#define kD 0
#define kIz 0
#define kFF 0.000156
#define kMaxOutput = 1
#define kMinOutput = -1;

#define kMaxVel 2000
#define kMinVel 0
#define kMaxAcc 1500
#define kAllErr 0

// 
#define kP 5e-5
#define kI 1e-6
#define kD 0
#define kIz 0
#define kFF 0.000156
#define kMaxOutput = 1
#define kMinOutput = -1;

#define kMaxVel 2000
#define kMinVel 0
#define kMaxAcc 1500
#define kAllErr 0

class InitMotors {
    public:
        enum motorTypes
        {
            driveMotor = 1, 
            elevatorMotor = 2, 
            armMotor = 3, 
            grabberMotor = 4 
        };

        static void initMotor(rev::CANSparkMax* motor, bool invert, motorTypes mType) {
            //have an if for type and set different values on them based on what they should be
            m_motor->RestoreFactoryDefaults();
            motor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
            motor->SetInverted(invert);
            motor->SetSmartCurrentLimit(motorInitMaxCurrent);
            motor->SetSecondaryCurrentLimit(motorInitRatedCurrent, motorInitLimitCycles);

        }
        static void initDriveMotor(rev::CANSparkMax* motor, rev::CANSparkMax* follower, bool invert) {
            motor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
            motor->SetInverted(invert);
            follower->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
            follower->Follow(*motor, false);
            motor->SetSmartCurrentLimit(motorInitMaxCurrent);
            motor->SetSecondaryCurrentLimit(motorInitRatedCurrent, motorInitLimitCycles);
            follower->SetSmartCurrentLimit(motorInitMaxCurrent);
            motor->SetSecondaryCurrentLimit(motorInitRatedCurrent, motorInitLimitCycles);
            
        }

        static void initPID(rev::SparkMaxPIDController& m_pidController, motorTypes mType) {
            if(mType == driveMotor) {
                m_pidController.SetP(kP);
                m_pidController.SetI(kI);
                m_pidController.SetD(kD);
                m_pidController.SetIZone(kIz);
                m_pidController.SetFF(kFF);
                m_pidController.SetOutputRange(kMinOutput, kMaxOutput);
                m_pidController.SetSmartMotionMaxVelocity(kMaxVel);
                m_pidController.SetSmartMotionMinOutputVelocity(kMinVel);
                m_pidController.SetSmartMotionMaxAccel(kMaxAcc);
            }
        }

        static void setConversionFactors() {
            //comment takes in encoders and sets their conversion factors
        }

        static void initSingleMotor(rev::CANSparkMax* motor) {
            motor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        }

    private:


};