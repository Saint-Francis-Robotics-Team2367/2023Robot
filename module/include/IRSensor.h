#include "frc/AnalogInput.h"
#include <cmath>
#include <frc/SmartDashboard/SmartDashboard.h>

class IRSensor{
    public:
    IRSensor(int);
    double polemaxdist = 25;
    frc::AnalogInput *irsensor = NULL;
    int port;
    double getCM();
    bool isPole(double);
    private:
};
