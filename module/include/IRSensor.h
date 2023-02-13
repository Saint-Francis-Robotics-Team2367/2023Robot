#include "frc/AnalogInput.h"
#include <cmath>
#include <frc/SmartDashboard/SmartDashboard.h>
#include <optional>

class IRSensor{
    public:
    IRSensor(int);
    double polemaxdist = 25;
    double front2dist = 8;
    std::optional<frc::AnalogInput> irsensor;
    int port;
    double getCM();
    bool isPole(double);
    static double getAngle(double ir1, double ir2);
    private:
};
