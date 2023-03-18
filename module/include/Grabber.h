#include <rev/CANSparkMax.h>
#include <Macros.h>

#pragma once

class Grabber {
    public:
    rev::CANSparkMax* grabberMotor = new rev::CANSparkMax(grabberID, rev::CANSparkMax::MotorType::kBrushed);

    bool state = true;// T = closed, F = open

    int tripleState = 0;

    void Init() {
        grabberMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
        grabberMotor->SetSmartCurrentLimit(5);
    }

    void open(bool button) {
        if (button) {
            grabberMotor->Set(-1.0);
        } else {
            grabberMotor->Set(0);
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
            set(1.0);
        } else {
            set(-1.0);
        }
        

    }

    void set(double inp) {
        grabberMotor->Set(inp);
    }

    private:
};