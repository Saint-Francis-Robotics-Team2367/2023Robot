#include "ColorSensor.h"

ColorSensor::ColorSensor() {
}

rev::ColorSensorV3::RawColor ColorSensor::getColor(){
    rev::ColorSensorV3::RawColor rawColor = colorSensor.GetRawColor();
    
    frc::SmartDashboard::PutNumber("R", rawColor.red);
    frc::SmartDashboard::PutNumber("G", rawColor.green);
    frc::SmartDashboard::PutNumber("B", rawColor.blue);
    return rawColor;
}

uint32_t ColorSensor::getProximity(){
    uint32_t proximity = colorSensor.GetProximity();
    frc::SmartDashboard::PutNumber("Proximity", proximity);
    return proximity;
}

double ColorSensor::getIR(){
    double ir = colorSensor.GetIR();
    frc::SmartDashboard::PutNumber("IR", ir);
    return ir;
}

bool ColorSensor::alignedIR(double ir){
    bool aligned = ir > minIR;
    frc::SmartDashboard::PutBoolean("IR Aligned", aligned);
    return aligned;
}

bool ColorSensor::matchesCubeColor(rev::ColorSensorV3::RawColor matchColor){
    bool match = matchesTarget(matchColor, cubeColorTarget);
    frc::SmartDashboard::PutBoolean("Cube", match);
    return match;
}

bool ColorSensor::matchesConeColor(rev::ColorSensorV3::RawColor matchColor){
    bool match = matchesTarget(matchColor, coneColorTarget);
    frc::SmartDashboard::PutBoolean("Cone", match);
    return match;
}

bool ColorSensor::matchesTarget(rev::ColorSensorV3::RawColor rawColor, ColorSensor::colorTarget target){
    double max = std::max(std::max(rawColor.red, rawColor.blue), rawColor.green);
    double factor = max > 255? max : 255;
    frc::Color color = frc::Color(((double)rawColor.red)/(factor), 
                                  ((double)rawColor.green)/(factor), 
                                  ((double)rawColor.blue)/(factor));
    double total = (rawColor.red + rawColor.green + rawColor.blue)/3;
    std::array<int, 3> hsv = ColorSensor::rgbTohsv(color);
    frc::SmartDashboard::PutNumber("H", hsv[0]);
    frc::SmartDashboard::PutNumber("S", hsv[1]);
    frc::SmartDashboard::PutNumber("V", hsv[2]);


    bool match = target.minH < hsv[0] < target.maxH &&
       hsv[1] > target.minS &&
       hsv[2] > target.minV &&
       total > target.minTotal;

    return match;
}


std::array<int, 3> ColorSensor::rgbTohsv(frc::Color color){
    double r = color.red;
    double g = color.green;
    double b = color.blue;

    double max = std::max(std::max(r, g), b);
    double min = std::min(std::min(r, g), b);

    double h, s, v;
    v = max;

    double d = max - min;
    s = (max == 0) ? 0 : d / max;

    if (max == min) {
        h = 0;
    } else {
        if (max == r) {
            h = (g - b) / d + (g < b ? 6 : 0);
        } else if (max == g) {
            h = (b - r) / d + 2;
        } else if (max == b) {
            h = (r - g) / d + 4;
        }
        h /= 6;
    }

    return {h, s, v};
}
