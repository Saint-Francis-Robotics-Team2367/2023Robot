#include "Macros.h"
#include <rev/CANSparkMax.h>
#include <frc/Joystick.h>
#include <frc/SmartDashboard/SmartDashboard.h>


class IntakeModule {
    public:
    //CHECK THIS WITH ID'S


    // if you don't include getEncoder here, it doesn't build?
    rev::SparkMaxRelativeEncoder rotaryIntakeEncoder = rotaryIntake->GetEncoder();
    rev::SparkMaxPIDController rotaryIntakePIDController = rotaryIntake->GetPIDController();

    bool state = true;// T = closed, F = open
    double openState = -18;
    double closedState = 0;
    int tripleState = 0;

     void Init() {
        rotaryIntake->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast); //coast
        rotaryIntake->SetSmartCurrentLimit(5);
        rotaryIntakePIDController.SetP(1.3);
        rotaryIntakePIDController.SetI(0);
        rotaryIntakePIDController.SetD(0);
        rotaryIntakeEncoder.SetPosition(0);

        leftStar->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        leftStar->SetSmartCurrentLimit(5);
        rightStar->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        rightStar->SetSmartCurrentLimit(5);
        roller->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        roller->SetSmartCurrentLimit(5);
    }

    void toggle(bool buttonPressed) {
        if (buttonPressed) {
            tripleState += 1;
            tripleState = tripleState % 3;
        }
        if(tripleState == 0) {
            rotaryIntake->StopMotor();
            roller->StopMotor();
            leftStar->StopMotor();
            rightStar->StopMotor();
        } else if (tripleState == 1) {
            //set(1.0);
            //rotaryIntakePIDController.SetReference(openState, rev::CANSparkMax::ControlType::kPosition);
            roller->Set(-1);
            leftStar->Set(-1);
            rightStar->Set(-1);
        } else {
            //set(-1.0);
           //rotaryIntakePIDController.SetReference(closedState, rev::CANSparkMax::ControlType::kPosition);
            roller->Set(1);
            leftStar->Set(1);
            rightStar->Set(1);
        }
    }

    private:
};