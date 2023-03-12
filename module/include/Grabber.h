#include <rev/CANSparkMax.h>
#include <Macros.h>

#pragma once

class Grabber {
    public:
    rev::CANSparkMax* grabberMotor = new rev::CANSparkMax(grabberID, rev::CANSparkMax::MotorType::kBrushed);;

    bool state = true;// T = closed, F = open

    void Init() {
        grabberMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        grabberMotor->SetSmartCurrentLimit(5);
    }

    void set(double inp) {
        grabberMotor->Set(inp);
    }

    private:
};