#include <rev/CANSparkMax.h>
#include <Macros.h>

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
        while(grabSwitch.Get() == false) {
            grabberMotor->Set(-1.0);
        }
        grabberMotor->Set(0);
        grab_enc.SetPosition(openState);
    }

    void toggleTwo(bool buttonPressed) {
        if (buttonPressed) {
            doubleState += 1;
            doubleState = doubleState % 2;
        }
        if(tripleState == 0) {
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