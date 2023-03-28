#include <rev/CANSparkMax.h>
#include <Macros.h>
#include <frc/Timer.h>
#include <frc/SmartDashboard/SmartDashboard.h>

#pragma once

class Grabber
{
public:
    rev::CANSparkMax *grabberMotor = new rev::CANSparkMax(grabberID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxPIDController grabberPID = grabberMotor->GetPIDController();
    rev::SparkMaxRelativeEncoder grab_enc = grabberMotor->GetEncoder();
    rev::SparkMaxLimitSwitch grabSwitch = grabberMotor->GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen);

    // 0 - closed, 1 - open
    int state = 0;

    void Init()
    {
        grabberPID.SetP(1.3);
        grabberPID.SetI(0);
        grabberPID.SetD(0);
        grab_enc.SetPosition(0);
        grabberMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        grabberMotor->SetSmartCurrentLimit(20);
        grabberMotor->SetSecondaryCurrentLimit(20, 0);
    }

    void delay(int n) 
    {
        auto start = std::chrono::system_clock::now();
        auto end = start + std::chrono::seconds(n);
        std::chrono::duration<double> elapsed;
        do {
            elapsed = end - std::chrono::system_clock::now();
        } while (elapsed.count() > 0);
    }

    void openAuto() 
    {
        delay(2);
        state = 1;
        grabberMotor->Set(-1.0);
    }

    void closeAuto(){
        delay(2);
        state = 0;
        grabberMotor->Set(0);
    }

private:
};