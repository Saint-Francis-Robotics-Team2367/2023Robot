#include "ElevatorModule.h"

ElevatorModule::ElevatorModule(frc::XboxController* controller) { //pass in joystick too
    //m_ID = motorID;
    ctr = controller;
    elevatorThread = std::thread(&ElevatorModule::run, this); //initializing thread so can detach in robot init
}


double ElevatorModule::manualMove(double Linput, double Rinput) {
    //frc::SmartDashboard::PutNumber("l", Linput);
    //frc::SmartDashboard::PutNumber("r", Rinput);

    double L = Linput;
    double R = Rinput;
    if (Linput < triggerDeadband) {L = 0;}
    if (Rinput < triggerDeadband) {R = 0;}
    return (R - L) / slowCoefficient; 
}

double ElevatorModule::getPos() {
    return enc.GetPosition();
}

double ElevatorModule::getHeight() {
    return height;
}

void ElevatorModule::resetPos() {
    height = 0;
}

void ElevatorModule::setPos(double setpoint) {
    setpoint = std::clamp(setpoint, kElevatorMinHeight, kElevatorMaxHeight); //clamps b/t two pts
    frc::SmartDashboard::PutNumber("inSetPos", setpoint);
    if(setpoint > height) {
        elevatorPID.SetP(pUp);
        elevatorPID.SetD(dUp);
    } else {
        elevatorPID.SetP(pDown);
        elevatorPID.SetD(dDown);
    }

    elevatorPID.SetReference(setpoint, rev::CANSparkMax::ControlType::kPosition);
    height = setpoint; 
}

void ElevatorModule::Init() {
   //elevatorMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    elevatorPID.SetP(pUp);
    elevatorPID.SetD(dUp);
    //enc.SetPosition(0);
    //resetPos();
    height = enc.GetPosition(); //don't close dashboard
}


void ElevatorModule::TeleopPeriodic(double Linput, double Rinput) {
        frc::SmartDashboard::PutNumber("position", getPos());
        frc::SmartDashboard::PutNumber("height", height);
        double output = manualMove(Linput, Rinput);
        // elevatorMotor->Set(output);
        // height = getPos();
        setPos(height + output);
        
}

void ElevatorModule::AutoPeriodic() {
        double set = frc::SmartDashboard::GetNumber("setpoint", 10);
        setPos(set);
        frc::SmartDashboard::PutNumber("setpoint", set);

        frc::SmartDashboard::PutNumber("position", getPos());
        frc::SmartDashboard::PutNumber("height", height);
}

void ElevatorModule::runInit() {

}

void ElevatorModule::run() {
    runInit();
    while(true) {
        auto nextRun = std::chrono::steady_clock::now() + std::chrono::milliseconds(5); //change milliseconds at telop
        frc::SmartDashboard::PutBoolean("elevator module", true);
        frc::SmartDashboard::PutNumber("elev left y", ctr->GetLeftY());
        std::this_thread::sleep_until(nextRun);
    }

}
