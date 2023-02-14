#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>
#include <frc/AnalogInput.h>
#include <rev/CANDigitalInput.h>
#include <rev/SparkMaxLimitSwitch.h>

class LimitSwitch{
    public:
    LimitSwitch(int);
    frc::DigitalInput *digitalInput;  
    frc::AnalogInput *analogInput;
    rev::SparkMaxLimitSwitch * limitSwitch;
    bool limit();
    private:
};
