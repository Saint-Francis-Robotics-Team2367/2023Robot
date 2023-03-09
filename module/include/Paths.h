#include <frc/TimedRobot.h>
#include <frc/SmartDashboard/SendableChooser.h>
#include <string> 

// straight paths (s): x,y 
// turn paths (r): angle, radius 
// false/true determine keeping velocity 
std::string defaultPath = "s 0,1,false|"; 
std::string path1 = "s 0,1,false|r -45,1,false|r 60,2,false|s 0,0,false|"; 
// can add more paths with path2, path3, etc. 

char delimiter = '|';

// choosing an autonomous path 
frc::SendableChooser<std::string> chooser;
const std::string autoDefault = defaultPath;
const std::string autoCustom = path1; // define more variables if there are more custom paths 
std::string selected;







