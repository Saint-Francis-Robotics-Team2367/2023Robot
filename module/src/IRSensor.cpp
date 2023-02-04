#include "IRSensor.h"

IRSensor::IRSensor(int port){
    this->port = port;
    irsensor = frc::AnalogInput(port);
}

double IRSensor::getCM(){
    double ir = (pow(irsensor->GetAverageVoltage(), -1.2045)) * 27.726;
    frc::SmartDashboard::PutNumber("IR", ir);
    return ir;
}

bool IRSensor::isPole(double ir){
    bool pole = ir <= polemaxdist;
    frc::SmartDashboard::PutBoolean("Pole", pole);
    return pole;
}