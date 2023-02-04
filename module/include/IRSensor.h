#include "frc/AnalogInput.h"
#include <cmath>
#include <frc/SmartDashboard/SmartDashboard.h>

class IRSensor{
    public:
    IRSensor(int);
    double polemaxdist = 25;
    std::optional<frc::AnalogInput> irsensor;
    int port;
    double getCM();
    bool isPole(double);
    private:
};
