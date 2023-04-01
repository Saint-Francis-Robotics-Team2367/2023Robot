#include "ElevatorModule.h"
#include "ShuffleUI.h"

ElevatorModule::ElevatorModule(frc::XboxController* controller, frc::XboxController* controllerOperator) { //pass in joystick too
    //m_ID = motorID;
    ctr = controller;
    ctrOperator = controllerOperator;
    elevatorThread = std::thread(&ElevatorModule::run, this); //initializing thread so can detach in robot init
}


double ElevatorModule::manualMove(double Linput, double Rinput) {
    //frc::SmartDashboard::PutNumber("l", Linput);
    //frc::SmartDashboard::PutNumber("r", Rinput);
    //change to ShuffleUI if uncommented

    double L = Linput;
    double R = Rinput;
    if (fabs(Linput) < triggerDeadband) {L = 0;}
    if (Rinput < triggerDeadband) {R = 0;}
    return (L - R) / slowCoefficient; 
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
       
        ShuffleUI::MakeWidget("Elev pos", tab, currentPosition);


        //checking end case if current position exceeds 
        if(down) {
            if(currentPosition < setpoint) {   
                currentPosition = setpoint;
                ShuffleUI::MakeWidget("reducto", tab, currentPosition);
            }
        } else {
            if(currentPosition > setpoint) {
                currentPosition = setpoint;
            }
        }

        currentPosition = std::clamp(currentPosition, kElevatorMinHeight, kElevatorMaxHeight); 
        
        
        ShuffleUI::MakeWidget("setpoint", tab, setpoint);
        elevatorPID.SetReference(std::copysign(currentPosition, setpoint), rev::CANSparkMax::ControlType::kPosition); //setpoint uses encoder
        prevTime = frc::Timer::GetFPGATimestamp().value();
        ShuffleUI::MakeWidget("prevTime", tab, prevTime);
    }
      height = setpoint; 
      return true;
  }


void ElevatorModule::setPos(double setpoint) {
    setpoint = std::clamp(setpoint, kElevatorMinHeight, kElevatorMaxHeight); //clamps b/t two pts
    ShuffleUI::MakeWidget("inSetPos", tab, setpoint);
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
    elevatorMotor->SetSmartCurrentLimit(30); //Pranav gave me this number
}


void ElevatorModule::TeleopPeriodic(double Linput, double Rinput) {
        ShuffleUI::MakeWidget("position", tab, getPos());
        ShuffleUI::MakeWidget("height", tab, height);
        double output = manualMove(Linput, Rinput);
        // elevatorMotor->Set(output);
        // height = getPos();
        setPos(height + output);
}

void ElevatorModule::AutoPeriodic() {
        double set = ShuffleUI::GetDouble("setpoint", tab, 25);

        setPos(set, true);
        ShuffleUI::MakeWidget("setpoint", tab, set);

        ShuffleUI::MakeWidget("position", tab, getPos());
        ShuffleUI::MakeWidget("height", tab, height);
}

void ElevatorModule::autoSet(double setpoint) {
    autoAmount = setpoint;
    isRunningAuto = true;
}

void ElevatorModule::zeroAtTop() {
    //if (forward)
        elevatorMotor->Set(0.3);
}

void ElevatorModule::runInit() {

}

void ElevatorModule::run() {
    runInit();
    Init();
    int currCount = 0;
    while(true) {
        auto nextRun = std::chrono::steady_clock::now() + std::chrono::milliseconds(5); //change milliseconds at telop
       // elevatorMotor->Set(ctr->GetLeftTriggerAxis() - ctr->GetRightTriggerAxis());
        // ShuffleUI::MakeWidget("elevator module", tab, true);
        // ShuffleUI::MakeWidrotget("elev left y", tab, ctrOperator->GetLeftY());
       
        if(state == 't') {

             if(!currentlyMoving) {
                if(ctr->GetXButton()) { //|| (ShuffleUI::GetInt("Score", "ArmtoElev", -1) == topLeftPole || ShuffleUI::GetInt("Score", "ArmtoElev", -1) == topRightPole)
                currentlyMoving = true;
                setPos(kHighScoreHeight, true);
                ShuffleUI::MakeWidget("x pressed", tab, true);
                currentlyMoving = false;
            }

            if(ctr->GetYButton()) {
                currentlyMoving = true;
                setPos(kLowestHeight, true);
                ShuffleUI::MakeWidget("y pressed", tab, true);
                currentlyMoving = false;
            }
            }
            
            //need this line for down movement to work? also is manual, could replace triggers with 0, 0 and it works
            TeleopPeriodic(ctrOperator->GetLeftTriggerAxis(), ctrOperator->GetRightTriggerAxis()); //for some reason either need this or teleop periodic for moving downwards to work
            //elevatorMotor->Set(ctr->GetLeftTriggerAxis() - ctr->GetRightTriggerAxis());
        }
        if(state == 'a') {
            if(isRunningAuto) {
               isFinished = setPos(autoAmount, true);
               isRunningAuto = false;
            }
        }
        std::this_thread::sleep_until(nextRun);
    }

};
