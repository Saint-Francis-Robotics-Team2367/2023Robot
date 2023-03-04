#include "frc/AnalogInput.h"
#include <cmath>
#include <frc/SmartDashboard/SmartDashboard.h>
#include <optional>
#include <list>
#include <array>

class IRSensor{
    public:
    IRSensor(int);
    IRSensor(int port, int x, int y, int a);

    std::optional<frc::AnalogInput> irsensor;
    int port;
    double x;
    double y;
    double a;

    double polemaxdist = 25;
    double front2dist = 8;

    double getCM();
    bool isPole(double);

    static double getAngle(double ir1, double ir2);
    static std::array<double, 2> onFieldLocation(std::list<IRSensor>);
    private:
    
};
