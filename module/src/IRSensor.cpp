#include "IRSensor.h"
#include <cmath>

IRSensor::IRSensor(int port){
    this->port = port;
    irsensor = frc::AnalogInput(port);
}

double IRSensor::getCM1(){
    double ir = (pow(irsensor->GetAverageVoltage(), -1.2045)) * 27.726;
    frc::SmartDashboard::PutNumber("IR1", ir);
    return ir;
}
double IRSensor::getCM2(){
    double ir = (pow(irsensor->GetAverageVoltage(), -1.2045)) * 27.726;
    frc::SmartDashboard::PutNumber("IR2", ir);
    return ir;
}

bool IRSensor::isPole(double ir){
    bool pole = ir <= polemaxdist;
    frc::SmartDashboard::PutBoolean("Pole", pole);
    return pole;
}
