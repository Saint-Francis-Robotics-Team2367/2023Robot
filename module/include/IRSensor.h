#include "frc/AnalogInput.h"
#include <cmath>
#include <frc/SmartDashboard/SmartDashboard.h>

class IRSensor{
    public:
    int port = 0;
    double maxdist = 25;
    frc::AnalogInput irsensor{port};
    double getIR();
    bool isPole(double);
    private:
};
