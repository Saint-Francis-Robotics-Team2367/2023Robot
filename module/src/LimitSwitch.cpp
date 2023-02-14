#include "LimitSwitch.h"

LimitSwitch::LimitSwitch(int port){
    //limitSwitch = new rev::SparkMaxLimitSwitch();
    digitalInput = new frc::DigitalInput(port);
}

bool LimitSwitch::limit(){
    return digitalInput->Get();
}