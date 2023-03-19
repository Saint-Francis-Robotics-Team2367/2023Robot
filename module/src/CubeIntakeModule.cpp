#include "CubeIntakeModule.h"

CubeIntakeModule::CubeIntakeModule(frc::XboxController* controller, frc::XboxController* controllerOperator) {
    ctr = controller;
    ctrOperator = controllerOperator;
    cubeIntakeThread = std::thread(&CubeIntakeModule::run, this); //initializing thread so can detach in robot init
}

void CubeIntakeModule::toggleIntake() {
    double setpoint = 30;

    rotaryPID.SetOutputRange(-0.1, 0.1); //test
    if(intakeDown) { //original state this is false
        setpoint *= -1;
    } 
    //Motion Profile !?!?!?!?!?!?!?!?!??!!?!?!?!??!?!?!?!/!?!?!?!?!?!?!?
    rotaryPID.SetReference(setpoint, rev::ControlType::kPosition); 
    intakeDown = !intakeDown;
}

void CubeIntakeModule::toggleScoring(bool stop, bool isIntaking, int amount) {
    if(stop) {
        leftStar->StopMotor();
        rightStar->StopMotor();
        intakeRoller->StopMotor();
    } else if(isIntaking) {
        leftStar->Set(-1 * amount);
        rightStar->Set(-1 * amount);
        intakeRoller->Set(-1 * amount);
    } else {
        leftStar->Set(amount);
        rightStar->Set(amount);
        intakeRoller->Set(amount);
    }

}

void CubeIntakeModule::runInit() {
    //set break mode and current limits.
    rotary->SetSmartCurrentLimit(20);
    leftStar->SetSmartCurrentLimit(20);
    rightStar->SetSmartCurrentLimit(20);
    intakeRoller->SetSmartCurrentLimit(20);
    leftStar->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    rightStar->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    intakeRoller->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    rotary->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}

void CubeIntakeModule::run() {
    runInit();
    int state = 0;
    bool stop;
    bool isIntaking;
    while(true) {
        if(state == 't') {
            if(ctr->GetRightBumperPressed()) {
                toggleIntake();
            }

            if(ctrOperator->GetAButtonPressed()) {
                if(state == 0) {stop = true;}
                if(state == 1) {stop = false; isIntaking = true;}
                if(state == 2) {stop = false; isIntaking = false;}

                state += 1;
                state %= 3;
            }

            toggleScoring(stop, isIntaking,  ctrOperator->GetLeftY());
        }
    }
}


