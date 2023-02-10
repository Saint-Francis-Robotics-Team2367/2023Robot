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

static double getAngle(double ir1, double ir2) {
    double d = 8.0; //distance between IR sensors
    double result = atan(d/(abs(ir1-ir2))); //trigonometry
    frc::SmartDashboard::PutNumber("Angle", 90-result*(180/3.14159265398979)); //conversion to degrees
    return (90-result*(180/3.14159265398979)); //90 might have to be removed
}