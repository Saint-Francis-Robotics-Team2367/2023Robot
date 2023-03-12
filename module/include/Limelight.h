#include <vector>
#include <cmath>
#include <LimelightHelpers.h>
#include <frc/SmartDashboard/SmartDashboard.h>

#define PI 3.14159265359


class Limelight {
    public:

    //distance
    //Clean this up later...
    double X = 21.55491;
    double shortY = 8.65436;
    double longY = 25.37496;

    
    double shortD = (X * X) + (shortY * shortY);
    double longD = (X * X) + (longY * longY);
    double shortAngle = atan(shortY / X);
    double longAngle = atan(longY / X);

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


    std::vector<double> getTargetXY(double targetX, double targetY, double targetAngle, int pole) { //pole is 0, 1, 2, 3 for bottom right, bottom left, top right, top left
        //Assume we got x_target, y_target, angle to target

        double targetDistancePolar = polePolarArray.at(pole).at(0); //Depends on pole selected
        double targetAnglePolar = polePolarArray.at(pole).at(1); //Depends on pole selection

        double finalAngle = targetAngle + targetAnglePolar;
        double tagX = targetDistancePolar * cos(finalAngle);
        double tagY = targetDistancePolar * sin(finalAngle);

        return std::vector{targetX + tagX, targetY + tagY};
    }

    std::vector<double> getFieldPos(){
        std::vector<double> pose = LimelightHelpers::getBotpose("");
        return pose; //TX, TY, TZ, RX, RY, RZ
    }

    std::vector<double> getRetroreflectiveAngles(){
        double tx = LimelightHelpers::getTX("");
        double ty = LimelightHelpers::getTY("");
        return std::vector<double>{tx, ty};
    }

    void switchToPipeline(int pipeline){
        LimelightHelpers::setPipelineIndex("", pipeline);
    }

    int getPipeline(){
        return LimelightHelpers::getLimelightNTDouble("", "pipeline");
    }
    
    private:


};
