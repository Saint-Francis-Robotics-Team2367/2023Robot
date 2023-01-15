#include "ColorSensor.h"

ColorSensor::ColorSensor() {
    // for(frc::Color coneColor: coneColorTargets){
    //     coneColorMatcher.AddColorMatch(coneColor);
    // }

    // for(frc::Color cubeColor: cubeColorTargets){
    //     cubeColorMatcher.AddColorMatch(cubeColor);
    // }

    // coneColorMatcher.SetConfidenceThreshold(minMatchConfidence);
    // cubeColorMatcher.SetConfidenceThreshold(minMatchConfidence);
}

frc::Color ColorSensor::getColor(){
    frc::Color color = colorSensor.GetColor();
    frc::SmartDashboard::PutNumber("Red:ColorSensor", color.red);
    frc::SmartDashboard::PutNumber("Green:ColorSensor", color.green);
    frc::SmartDashboard::PutNumber("Blue:ColorSensor", color.blue);
    return color;
}

uint32_t ColorSensor::getProximity(){
    uint32_t proximity = colorSensor.GetProximity();
    frc::SmartDashboard::PutNumber("Proximity:ColorSensor", proximity);
    return proximity;
}

bool ColorSensor::matchesCubeColor(frc::Color matchColor){
    // double confidence = 0;
    // frc::Color matchedColor = cubeColorMatcher.MatchClosestColor(matchColor, confidence);
    // bool matched = cubeColorTargets.count(matchColor) > 0;
    // frc::SmartDashboard::PutNumber("CubeMatch:ColorSensor", matched);
    // frc::SmartDashboard::PutNumber("CubeMatchConfidence:ColorSensor", confidence);
    // return matched;
}

bool ColorSensor::matchesConeColor(frc::Color matchColor){
    // double confidence = 0;
    // frc::Color matchedColor = coneColorMatcher.MatchClosestColor(matchColor, confidence);
    // bool matched = coneColorTargets.count(matchColor) > 0;
    // frc::SmartDashboard::PutNumber("ConeMatch:ColorSensor", matched);
    // frc::SmartDashboard::PutNumber("ConeMatchConfidence:ColorSensor", confidence);
    // return matched;
}