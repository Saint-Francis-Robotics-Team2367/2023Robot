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

bool ElevatorModule::setPos(double setpoint, bool isMotionProfiled) {
    setpoint = std::clamp(setpoint, kElevatorMinHeight, kElevatorMaxHeight); //have this hear check
    float timeElapsed, distanceToDeccelerate, currentVelocity = 0.0; //currentPosition is the set point
    double currentPosition = getPos(); //current velocity is a class variable
    float prevTime = frc::Timer::GetFPGATimestamp().value();
    bool down = false;
    if(setpoint >= getPos()) { //could use height here too
        elevatorPID.SetP(pUp);
        elevatorPID.SetD(dUp);             
    } else {
        elevatorPID.SetP(pDown);
        elevatorPID.SetD(dDown);
        down = true;
    }
    while(fabs(currentPosition - setpoint) > 0){ //40 < 20, fix this on arm too!!!!!! while(fabs(currentPosition) < fabs(setpoint)){
        if(stopAuto) {
          break;
        }

        timeElapsed = frc::Timer::GetFPGATimestamp().value() - prevTime;
        distanceToDeccelerate = (3 * currentVelocity * currentVelocity) / (2 * maxAcc); //change
        if (fabs(distanceToDeccelerate) > fabs(setpoint - currentPosition)) {  //change this line???
          currentVelocity -= (maxAcc * timeElapsed);
        }
        else //increase velocity
        {
          currentVelocity += (maxAcc * timeElapsed);
          if (fabs(currentVelocity) > fabs(maxVelocity))
          {
            currentVelocity = maxVelocity;
          }
        }

        //adding or substracting positions
        if(down) {
            currentPosition -= currentVelocity * timeElapsed;
        } else {
             currentPosition += currentVelocity * timeElapsed;
        }
       
        frc::SmartDashboard::PutNumber("Elev pos", currentPosition);


        //checking end case if current position exceeds 
        if(down) {
            if(currentPosition < setpoint) {   
                currentPosition = setpoint;
                frc::SmartDashboard::PutNumber("reducto", currentPosition);
            }
        } else {
            if(currentPosition > setpoint) {
                currentPosition = setpoint;
            }
        }

        currentPosition = std::clamp(currentPosition, kElevatorMinHeight, kElevatorMaxHeight); 
        
        
        frc::SmartDashboard::PutNumber("setpoint", setpoint);
        elevatorPID.SetReference(std::copysign(currentPosition, setpoint), rev::CANSparkMax::ControlType::kPosition); //setpoint uses encoder
        prevTime = frc::Timer::GetFPGATimestamp().value();
        frc::SmartDashboard::PutNumber("prevTime", prevTime);
    }
      height = setpoint; 
      return true;
  }


void ElevatorModule::setPos(double setpoint) {
    setpoint = std::clamp(setpoint, kElevatorMinHeight, kElevatorMaxHeight); //clamps b/t two pts
    frc::SmartDashboard::PutNumber("inSetPos", setpoint);
    if(setpoint > getPos()) {
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
    enc.SetPosition(0);
    //resetPos();
    height = enc.GetPosition(); //don't close dashboard
    elevatorMotor->SetSmartCurrentLimit(40); //Pranav gave me this number
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
        double set = frc::SmartDashboard::GetNumber("setpoint", 25);

        setPos(set, true);
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
       
        if(state == 't') {
             if(!currentlyMoving) {
                if(ctr->GetXButtonPressed()) {
                currentlyMoving = true;
                setPos(kHighScoreHeight, true);
                frc::SmartDashboard::PutBoolean("x pressed", true);
                currentlyMoving = false;
            }

             if(ctr->GetAButton()) {
                currentlyMoving = true;
                setPos(kLowScoreHeight, true);
                frc::SmartDashboard::PutBoolean("B pressed", true);
                currentlyMoving = false;
            }

            if(ctr->GetYButton()) {
                currentlyMoving = true;
                setPos(kLowestHeight, true);
                frc::SmartDashboard::PutBoolean("y pressed", true);
                currentlyMoving = false;
            }
            }
            //need this line for down movement to work? also is manual, could replace triggers with 0, 0 and it works
            TeleopPeriodic(ctr->GetLeftTriggerAxis(), ctr->GetRightTriggerAxis()); //for some reason either need this or teleop periodic for moving downwards to work
            
        }
        if(state == 'a') {
            //TeleopPeriodic(ctr->GetLeftTriggerAxis(), ctr->GetRightTriggerAxis()); //for some reason either need this or teleop periodic for moving downwards to work

            // //AutoPeriodic();
            // if(test) {
            //     setPos(25, true);
            //     test = false;
            // }
            // if(oneRun && !test) {
            //     setPos(0, true);
            //     oneRun = false;
            // }
            // frc::SmartDashboard::PutNumber("left trigger", ctr->GetLeftTriggerAxis());
        }
        std::this_thread::sleep_until(nextRun);
    }

}
