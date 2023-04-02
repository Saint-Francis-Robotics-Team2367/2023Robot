#include <frc/SmartDashboard/SendableChooser.h>
#include <string> 

enum autoPathType
{
    turn = 't', 
    straight = 's', 
    elev = 'e', 
    arm = 'a' 
};

class autoPath {
    public:
        autoPathType action; 
        
        float dis; 
        float angle; 
        float radius; 
        bool keepVelocity;
        bool motionProfiling; 
        float setpoint; 

        // arm angles for pid 
        double outer; 
        double inner;

        autoPath(autoPathType type){
            action = type;
        };

        void register_turn(float ang, float r=0, bool keepV=false){
            keepVelocity = keepV; 
            radius = r;
            angle = ang; 
        };

        void register_straight(int d, bool keepV=false){
            keepVelocity = keepV; 
            dis = d; 
        };

        void register_elev(float point){
            setpoint = point; 
        };

        void register_arm(double o, double i){
            outer = o; 
            inner = i; 
        };

};



