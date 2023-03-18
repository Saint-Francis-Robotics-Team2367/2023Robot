#include <frc/SmartDashboard/SendableChooser.h>
#include <string> 

enum autoPathType
{
    turn = false,
    straight = true
};

class autoPath {
    public:
        autoPathType straight; 
        int dis; 
        int angle; 
        int radius; 
        bool keepVelocity;

        autoPath(autoPathType type, bool keepV=false){
            keepVelocity = keepV;
            straight = type;
        };

        void register_turn(int ang, int r=0){
            radius = r;
            angle = ang; 
        };

        void register_straight(int d){
            dis = d; 
        };
};

float angle = 0; 
float radius = 0; 

// choosing an autonomous path 
frc::SendableChooser<std::string> chooser;
const std::string autoDefault = "Default";
const std::string autoCustom = "Path 1"; // define more variables if there are more custom paths 
std::string selected;
