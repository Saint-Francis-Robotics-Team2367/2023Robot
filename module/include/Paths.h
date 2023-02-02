#include <frc/TimedRobot.h>
#include <frc/SmartDashboard/SendableChooser.h>
#include <string> 

// straight paths (s): x,y 
// turn paths (r): angle, radius 
// false/true determine keeping velocity 
std::string defaultPath = "r 90,10,false|s 2,2,false|r 45,3,false|s 10,2,false|"; 
std::string path1 = "r -45,10,false|"; 
// can add more paths with path2, path3, etc. 

char delimiter = '|';

// choosing an autonomous path 
frc::SendableChooser<std::string> chooser;
const std::string autoDefault = "Default";
const std::string autoCustom = path1; // may need to define more variables if there are more custom paths 
std::string selected;







