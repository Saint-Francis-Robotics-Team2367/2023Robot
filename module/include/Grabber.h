#include <rev/CANSparkMax.h>
#include <Macros.h>
#include <frc/Timer.h>

#pragma once

class Grabber {
    public:
    rev::CANSparkMax* grabberMotor = new rev::CANSparkMax(grabberID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxPIDController grabberPID = grabberMotor->GetPIDController();
    rev::SparkMaxRelativeEncoder grab_enc = grabberMotor->GetEncoder();
    rev::SparkMaxLimitSwitch grabSwitch = grabberMotor->GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen);

    bool state = true;// T = closed, F = open
    double openState = -18;
    double closedState = 0;
    int tripleState = 0;
    int doubleState = 0;

    void Init() {
        grabberPID.SetP(1.3);
        grabberPID.SetI(0);
        grabberPID.SetD(0);
        grab_enc.SetPosition(0);
        grabberMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        grabberMotor->SetSmartCurrentLimit(5);
        grabberMotor->SetSecondaryCurrentLimit(5, 0);
    }

    void open(bool button) {
        if (button) {
            grabberMotor->Set(-1.0);
        } else {
            grabberMotor->Set(0);
        }
        
    }

    void openAuto() {
        float startTime = frc::Timer::GetFPGATimestamp().value();
        while(true) {
            frc::SmartDashboard::PutNumber("grabberStuff", grabSwitch.Get());
            grabberMotor->Set(-1.0);
            if(frc::Timer::GetFPGATimestamp().value() - startTime > 3) {
                break;
                grabberMotor->Set(0);
                grab_enc.SetPosition(openState);
            }
        }
        grabberMotor->Set(0);
        grab_enc.SetPosition(openState);
    }

    void toggleTwo(bool buttonPressed) {
        if (buttonPressed) {
            doubleState += 1;
            doubleState = doubleState % 2;
        }
        if(doubleState == 0) {
            grabberPID.SetReference(openState, rev::CANSparkMax::ControlType::kPosition);
        }
        else {
            //set(-1.0);
            grabberPID.SetReference(closedState, rev::CANSparkMax::ControlType::kPosition);
        }
    }



    void toggle(bool buttonPressed) {
        if (buttonPressed) {
            tripleState += 1;
            tripleState = tripleState % 3;
        }
        if(tripleState == 0) {
            grabberMotor->StopMotor();
        } else if (tripleState == 1) {
            //set(1.0);
            grabberPID.SetReference(openState, rev::CANSparkMax::ControlType::kPosition);
        } else {
            //set(-1.0);
            grabberPID.SetReference(closedState, rev::CANSparkMax::ControlType::kPosition);
        }
        

    }

    void set(double inp) {
        grabberMotor->Set(inp);
    }

    private:
};