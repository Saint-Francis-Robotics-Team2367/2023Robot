#include "IRSensor.h"
#include <cmath>

IRSensor::IRSensor(int port){
    this->port = port;
    irsensor = frc::AnalogInput(port);
}

double IRSensor::getCM(){
    double ir = (pow(irsensor->GetAverageVoltage(), -1.2045)) * 27.726;
    return ir;
}

bool IRSensor::isPole(double ir){
    bool pole = ir <= polemaxdist;
    return pole;
}

double IRSensor::getAngle(double ir1, double ir2) {
    double d = 8.0;
    double result = atan(d/(abs(ir1-ir2)));
    frc::SmartDashboard::PutNumber("Angle", 90-result*(180/3.14159265398979));
    return (90-result*(180/3.14159265398979));
}