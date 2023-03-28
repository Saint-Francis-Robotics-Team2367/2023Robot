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

    void openAuto() {
        
    }

private:
};