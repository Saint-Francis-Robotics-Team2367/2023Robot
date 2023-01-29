#include "IRSensor.h"

double IRSensor::getIR(){
    double ir = (pow(irsensor.GetAverageVoltage(), -1.2045)) * 27.726;
    frc::SmartDashboard::PutNumber("IR", ir);
    return ir;
}

bool IRSensor::isPole(double ir){
    bool pole = ir <= maxdist;
    frc::SmartDashboard::PutBoolean("Pole", pole);
    return pole;
}