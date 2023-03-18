#include "CubeIntakeModule.h"

CubeIntakeModule::CubeIntakeModule(frc::XboxController* controller, frc::XboxController* controllerOperator) {
    ctr = controller;
    ctrOperator = controllerOperator;
    cubeIntakeThread = std::thread(&CubeIntakeModule::run, this); //initializing thread so can detach in robot init
}

void CubeIntakeModule::toggleIntake() {
    if(intakeDown) {

    }



    intakeDown = !intakeDown
}

void CubeIntakeModule::runInit() {

}

void CubeIntakeModule::run() {
    runInit();
    while(true) {
        if(state == 't') {

        }
    }
}


