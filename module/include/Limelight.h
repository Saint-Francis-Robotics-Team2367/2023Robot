#include <vector>
#include "networktables/NetworkTable.h"
#include <networktables/NetworkTableInstance.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>
#include <iostream>
#include <LimelightHelpers.h>
#include <frc/SmartDashboard/SmartDashboard.h>

#define PI 3.14159265359


class Limelight {
    public:

    enum poleIDs {
        bottomRightPole = 0,
        bottomLeftPole = 1,
        topRightPole = 2,
        topLeftPole = 3
    };

    struct Point {
        double x;
        double y;
    };
     
    struct polarPoint {
        double mag; //Inches
        double ang; //Degrees
    };

    //distance
    //Clean this up later...
    const double X = 21.55491;
    const double shortY = 8.65436;
    const double longY = 25.37496;

    
    const double shortD = sqrt((X * X) + (shortY * shortY));
    const double longD = sqrt((X * X) + (longY * longY));
    const double shortAngle = atan(shortY / X) * 180 / PI;
    const double longAngle = atan(longY / X) * 180 / PI;

    std::vector<std::vector<double> > polePolarArray { //pole is 0, 1, 2, 3 for bottom right, bottom left, top right, top left
        {shortD, shortAngle}, //length, angle
        {shortD, 180 - shortAngle},
        {longD, longAngle},
        {longD, 180 - longAngle},
        {} //18 in
    };

    std::vector<double> getTargetCoords() {
        // double px = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0.0);
        // double py = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty",0.0);
       
        double px = LimelightHelpers::getTX("");
        double py = LimelightHelpers::getTY("");

        return std::vector<double> {px, py};
    }

    std::vector<double> getTargetAngles(double in_x, double in_y) { // Gets ax, ay
        double px = in_x;
        double py = in_y;
        // double tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0.0);
        // double tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0.0);

        // frc::SmartDashboard::PutNumber("TX", px);
        // frc::SmartDashboard::PutNumber("TY", py);

        double nx = (1/160) * (px - 159.5);
        double ny = (1/120) * (119.5 - py);

        double horizontal_fov = 54;
        double vertical_fov = 41;

        double vpw = 2.0*tan(horizontal_fov/2);
        double vph = 2.0*tan(vertical_fov/2);

        double x = vpw/2 * nx;
        double y = vph/2 * ny;

        double ax = atan2(1,x);
        double ay = atan2(1,y);
        
        // double axr = (ax * PI)/180;
        // double ayr = (ay * PI)/180;

        // double d = 4;
        // double rx = d * tan(axr);
        // double ry = sqrt(d*d + rx*rx) * tan(ayr);

        // frc::SmartDashboard::PutNumber("AX", ax);
        // frc::SmartDashboard::PutNumber("AY", ay);
        // frc::SmartDashboard::PutNumber("RX", rx);
        // frc::SmartDashboard::PutNumber("RY", ry);
        return std::vector<double> {ax, ay};
    }

    //static std::vector<double> calculateTargetXY(double targetHeight, double LLheight) {
        
    //}


    Point getTargetXY(double targetX, double targetY, double targetAngle, int pole) { //pole is 0, 1, 2, 3 for bottom right, bottom left, top right, top left

        double tag_RelMagnitude = polePolarArray.at(pole).at(0);
        double tag_RelAngle = polePolarArray.at(pole).at(1) * PI / 180;
        double angleToTarg =  atan(fabs(targetY / targetX));
        targetAngle = targetAngle * PI / 180;

        double tapeAngle = (PI / 2) - angleToTarg - targetAngle;
        double tapeX = targetX + (tag_RelMagnitude * cos(tapeAngle + tag_RelAngle));
        double tapeY = targetY + (tag_RelMagnitude * sin(tapeAngle + tag_RelAngle));
        
        
        // targetAngle = targetAngle * PI / 180;
        // double tapeX = targetX + (tag_RelMagnitude * cos(angleToTarg - targetAngle + tag_RelAngle));
        // double tapeY = targetY + (tag_RelMagnitude * sin(angleToTarg - targetAngle + tag_RelAngle));

        return Point{tapeX, tapeY};
    }

    std::vector<double> getFieldPos(){
        std::vector<double> pose = LimelightHelpers::getBotpose("");
        return pose; //TX, TY, TZ, RX, RY, RZ
    }

    std::vector<double> getTargetPoseRobotSpace() {
        //std::vector<double> pose = LimelightHelpers::getTargetPose_RobotSpace("limelight");
        //Above code returned an empty vector for some reason
        std::vector<double> pose = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumberArray("targetpose_robotspace", std::span<double>{});
        return pose;
    }

    std::vector<double> getRetroreflectiveAngles(){
        double tx = LimelightHelpers::getTX("");
        double ty = LimelightHelpers::getTY("");
        return std::vector<double>{tx, ty};
    }

    void switchToPipeline(int pipeline){
        // LimelightHelpers::setPipelineIndex("", pipeline);
        int current = getPipeline();
        nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("pipeline", pipeline);
    }

    int getPipeline(){
        return nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("pipeline", -1);
    }

    int getTargetDetected() {
       return nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv", 0);
    }
    
    private:


};
