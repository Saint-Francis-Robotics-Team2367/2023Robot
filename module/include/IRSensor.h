#include "frc/AnalogInput.h"
#include <cmath>
#include <frc/SmartDashboard/SmartDashboard.h>
#include <optional>

class IRSensor{
    public:
    IRSensor(int);
    double polemaxdist = 25;
    std::optional<frc::AnalogInput> irsensor;
    int port;
    double getCM1();
    double getCM2();
    bool isPole(double);
    private:
};
