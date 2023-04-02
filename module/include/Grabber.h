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
    rev::SparkMaxLimitSwitch reverseSwitch = grabberMotor->GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen);
    rev::SparkMaxLimitSwitch forwardSwitch = grabberMotor->GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen);

    // 0 - closed, 1 - open
    int state = 0;
    double range = 18.0;

    void Init()
    {
        grabberPID.SetP(1.3);
        grabberPID.SetI(0);
        grabberPID.SetD(0);
        grab_enc.SetPosition(0);
        grabberMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        grabberMotor->SetSmartCurrentLimit(20);
        grabberPID.SetOutputRange(-1, 1);
        forwardSwitch.EnableLimitSwitch(false);
        reverseSwitch.EnableLimitSwitch(true);
    }

   
    void open() {
        grabberPID.SetReference(-range, rev::CANSparkMax::ControlType::kPosition);
    }

    void close() {
        grabberPID.SetReference(0, rev::CANSparkMax::ControlType::kPosition);       
    }

    void togglePID() {
        state += 1;
        state = state % 2;
        if (state == 0) {
            grabberPID.SetReference(-0.1, rev::CANSparkMax::ControlType::kPosition);
        } else if (state == 1) {
            grabberPID.SetReference(-18, rev::CANSparkMax::ControlType::kPosition);
        }

    }

    void goToPID(double input) {
        grabberPID.SetReference(input, rev::CANSparkMax::ControlType::kPosition);
    }

private:
};