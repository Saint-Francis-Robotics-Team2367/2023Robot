#include "DriveBaseModule.h"
#include <iostream>
#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/NetworkTableEntry.h>
#include "ShuffleUI.h"

bool DriveBaseModule::initDriveMotor(rev::CANSparkMax* motor, rev::CANSparkMax* follower, bool invert) {
  motor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  motor->SetInverted(invert);
  follower->Follow(*motor, false);
  return motor->GetLastError() == rev::REVLibError::kOk;
}

bool DriveBaseModule::setPowerBudget(rev::CANSparkMax* motor, float iPeak, float iRated, int limitCycles) {
  motor->SetSmartCurrentLimit(iRated);
  motor->SetSecondaryCurrentLimit(iPeak, limitCycles);
  return motor->GetLastError() == rev::REVLibError::kOk;
}

bool DriveBaseModule::setDriveCurrLimit(float iPeak, float iRated, int limitCycles) {
  bool setlFront = setPowerBudget(lMotor, iPeak, iRated, limitCycles);
  bool setrFront = setPowerBudget(rMotor, iPeak, iRated, limitCycles);
  bool setlBack = setPowerBudget(lMotorFollower, iPeak, iRated, limitCycles);
  bool setrBack = setPowerBudget(rMotorFollower, iPeak, iRated, limitCycles);

  return setlFront && setrFront && setlBack && setrBack; // Failure on false
}

void DriveBaseModule::LimitRate(double& s, double& t) {
    double k = 6; //1/k = rate to speed up [so 0.2 seconds]
    double currTime = frc::Timer::GetFPGATimestamp().value();
    double deltaTime = currTime - prevTime;
    double r_s = (s - prev_value_speed) / deltaTime;
    double r_t = (t - prev_value_turn) / deltaTime;

    if ((fabs(r_s) > k) && (fabs(s) > fabs(prev_value_speed))){
        s = ((k * (fabs(r_s) / r_s) * deltaTime) + prev_value_speed);
    }
    if ((fabs(r_t) > k) && (fabs(t) > fabs(prev_value_turn))){
        t = ((k * (fabs(r_t) / r_t) * deltaTime) + prev_value_turn);
    }

    prev_value_speed = s;
    prev_value_turn = t;
    prevTime = currTime;
}

double DriveBaseModule::skim(double v) { //this for each motor to shave some stuff off on each side
  if (v > 1.0) {
    return -((v - 1.0) * driveTurningGain);
  } else if (v < -1.0) {
    return -((v + 1.0) * driveTurningGain);
  } return 0; 
}

//need to TEST SKIM CONSTANT
void DriveBaseModule::arcadeDrive(double xSpeedi, double zRotationi) {
    if (fabs(xSpeedi) < xDeadband)
        xSpeedi = 0;

    if (fabs(zRotationi) < yDeadband)
        zRotationi = 0;

    double xSpeed = std::copysign(pow(fabs(xSpeedi), 1.8), xSpeedi);
    double zRotation = std::copysign(pow(fabs(zRotationi), 2), zRotationi);

    LimitRate(xSpeed, zRotation);

    double leftMotorOutput = xSpeed + zRotation;
    double rightMotorOutput = xSpeed - zRotation; //removed turnSense need test

    if (leftMotorOutput != 0)
        leftMotorOutput = std::copysign((1/(1-xDeadband)) * fabs(leftMotorOutput) - (xDeadband/(1/xDeadband)), leftMotorOutput);
        
    if (rightMotorOutput != 0)
        rightMotorOutput = std::copysign((1/(1-yDeadband)) * fabs(rightMotorOutput) - (yDeadband/(1/yDeadband)), rightMotorOutput);

    //IN TESTING IN ACCORDANCE WITH THE GYRO DRIVE, CHECKK
    //leftMotorOutput += skim(rightMotorOutput); //NEED TO TEST ASKKKK
    //rightMotorOutput += skim(leftMotorOutput); //NEED TO TEST ASKKKK

    leftMotorOutput = std::clamp(leftMotorOutput, -1.0, 1.0);
    rightMotorOutput = std::clamp(rightMotorOutput, -1.0, 1.0);
    lMotor->Set(leftMotorOutput);
    rMotor->Set(rightMotorOutput); //try SetVoltage
}

void DriveBaseModule::gyroDriving() {
  float rightStickOutput = driverStick->GetRawAxis(4);
  float calculation = rightStickPID->Calculate(ahrs->GetRate()/150, rightStickOutput); //add skim
  arcadeDrive(driverStick->GetRawAxis(1) * (-1), calculation);
  //frc::SmartDashboard::PutNumber("output", calculation);
  ShuffleUI::MakeWidget("output", "PLACEHOLDER", calculation);
  //frc::SmartDashboard::PutNumber("gyro", ahrs->GetRate());
  ShuffleUI::MakeWidget("output", "PLACEHOLDER", ahrs->GetRate());

}

void DriveBaseModule::PIDTuning() {
  //double prevTime = frc::Timer::GetFPGATimestamp().value();
  double currentLeftLead = lMotor->GetOutputCurrent();
  double currentRightLead = rMotor->GetOutputCurrent();

  double voltageOverall = lMotor->GetBusVoltage() + rMotor->GetBusVoltage();
  //do getBusVoltage (returns voltage fed into motor controller)


  //frc::SmartDashboard::PutNumber("Total Current", currentLeftLead+currentRightLead);
  ShuffleUI::MakeWidget("Total Current", "PID Tuning", currentLeftLead+currentRightLead);
  //frc::SmartDashboard::PutNumber("Total Voltage", voltageOverall);
  ShuffleUI::MakeWidget("Total Voltage", "PID Tuning", voltageOverall);

  //Making it so you can manually set m_p and positionTotal: m_p is essential with PID, change by an order of magnitude to start run
  //double m_P = frc::SmartDashboard::GetNumber("Pd", 1);
  //bool isNegative;

  nt::GenericEntry *pSlider = ShuffleUI::MakeSlider("P Value", "PID Tuning", -1, 2, 0.0);
  double m_P = pSlider->GetDouble(0.0);
  lPID.SetP(m_P);
  rPID.SetP(m_P);
  //frc::SmartDashboard::PutNumber("Pd", m_P);

  //double m_D = frc::SmartDashboard::GetNumber("D Value", 0);
  //bool isNegative;

  nt::GenericEntry *dSlider = ShuffleUI::MakeSlider("D Value", "PID Tuning", -1, 2, 0.0);
  double m_D = dSlider->GetDouble(0.0);
  lPID.SetD(m_D);
  rPID.SetD(m_D);
  //frc::SmartDashboard::PutNumber("D Value", m_D);

  //double m_I = frc::SmartDashboard::GetNumber("I Value", 0);
  //bool isNegative;

  nt::GenericEntry *iSlider = ShuffleUI::MakeSlider("I Value", "PID Tuning", -1, 2, 0.0);
  double m_I = iSlider->GetDouble(0.0);
  lPID.SetI(m_I);
  rPID.SetI(m_I);
  //frc::SmartDashboard::PutNumber("I Value", m_I);

  //double I_Zone = frc::SmartDashboard::GetNumber("I_Zone", 0);

  nt::GenericEntry *iZoneSlider = ShuffleUI::MakeSlider("I Zone", "PID Tuning", -1, 2, 0.0);
  double I_Zone = iZoneSlider->GetDouble(0.0);

  //bool isNegative;
  lPID.SetIZone(I_Zone);
  rPID.SetIZone(I_Zone);
  //frc::SmartDashboard::PutNumber("I_Zone", I_Zone);

  lPID.SetIZone(I_Zone);

  //double waitTime = frc::SmartDashboard::GetNumber("waitTime", 4);
  //frc::SmartDashboard::PutNumber("waitTime", waitTime);

  nt::GenericEntry *waitTimeSlider = ShuffleUI::MakeSlider("Wait Time", "PID Tuning", -5, 5, 0.0);
  double waitTime = waitTimeSlider->GetDouble(0.0);

  double currTime = frc::Timer::GetFPGATimestamp().value();
  //frc::SmartDashboard::PutNumber("currTime", currTime);
  ShuffleUI::MakeWidget("currTime", "PID Tuning", currTime);
  //frc::SmartDashboard::PutNumber("Setpoint", delta);
  ShuffleUI::MakeWidget("Setpoint", "PID Tuning", delta);
  if(currTime > tuningPrevTime + waitTime) {
   
      lPID.SetReference(delta, rev::ControlType::kPosition);
      rPID.SetReference(delta, rev::ControlType::kPosition);
      delta = delta * -1.0;
 
      //frc::SmartDashboard::PutNumber("Right Encoder", rEncoder.GetPosition());
      ShuffleUI::MakeWidget("Right Encoder", "PID Tuning", rEncoder.GetPosition());
      //frc::SmartDashboard::PutNumber("Left Encoder", lEncoder.GetPosition());
      ShuffleUI::MakeWidget("Left Encoder", "PID Tuning", lEncoder.GetPosition());

      tuningPrevTime = frc::Timer::GetFPGATimestamp().value();
      //frc::SmartDashboard::PutNumber("prev time", tuningPrevTime);
  }
}

void DriveBaseModule::driveBaseTuning() {

  // double voltageOverall = lMotor->GetBusVoltage() + rMotor->GetBusVoltage();
  // frc::SmartDashboard::PutNumber("Total Voltage", voltageOverall);

  // frc::SmartDashboard::PutNumber("Total Current", lMotor->GetOutputCurrent()+rMotor->GetOutputCurrent());

  // double d_P = frc::SmartDashboard::GetNumber("d_P", rightStickPID.GetP());
  // frc::SmartDashboard::PutNumber("d_P", d_P);
  // rightStickPID.SetP(d_P);

  // double d_D = frc::SmartDashboard::GetNumber("d_D", rightStickPID.GetD());
  // frc::SmartDashboard::PutNumber("d_D", d_D);
  // rightStickPID.SetD(d_D);

  // double amountForward = frc::SmartDashboard::GetNumber("amtForward", 0);
  // frc::SmartDashboard::PutNumber("amtForward", amountForward);

  // if(driverStick->GetRawButton(1)) {
  //   frc::SmartDashboard::PutBoolean("Button Pressed", true);
  //   rightStickPID.SetSetpoint(0.3);
  //   arcadeDrive(amountForward, GetOutput());
  //   frc::SmartDashboard::PutNumber("output", GetOutput());
  // } else {
  //   frc::SmartDashboard::PutBoolean("Button Pressed", false);
  //   rightStickPID.SetSetpoint(0);
  //   arcadeDrive(0, 0);
  //   lMotor->StopMotor();
  //   rMotor->StopMotor();
  //   frc::SmartDashboard::PutNumber("output", GetOutput());
  // }
}

bool DriveBaseModule::PIDDrive(float totalFeet, bool keepVelocity) {
  //forward movement only *implement backwards movement with if statement if necessary
  float timeElapsed, distanceToDeccelerate, setpoint = 0.0; //currentPosition is the set point
  double currentPosition = 0; //current velocity is a class variable
  float prevTime = frc::Timer::GetFPGATimestamp().value();

  rEncoder.SetPosition(0);
  lEncoder.SetPosition(0);
  //frc::SmartDashboard::PutBoolean("inPIDDrive", true); 
  ShuffleUI::MakeWidget("inPIDDrive", "PLACEHOLDER", true);

  if(keepVelocity) {
    while(fabs(currentPosition) < fabs(totalFeet)) {
      if(stopAuto) {
        break;
      }
      //frc::SmartDashboard::PutNumber("lEncoder", lEncoder.GetPosition());
      ShuffleUI::MakeWidget("lEncoder", "PLACEHOLDER", lEncoder.GetPosition());
      //frc::SmartDashboard::PutNumber("rEncoder", rEncoder.GetPosition());
      ShuffleUI::MakeWidget("rEncoder", "PLACEHOLDER", rEncoder.GetPosition());
      timeElapsed = frc::Timer::GetFPGATimestamp().value() - prevTime;

      currentVelocity += maxAcc * timeElapsed;
      if (fabs(currentVelocity) > fabs(maxVelocity)) {
        currentVelocity = maxVelocity;
      }
      currentPosition += currentVelocity * timeElapsed;
      if(fabs(currentPosition) > fabs(totalFeet)) {
          currentPosition = totalFeet;
      }


      setpoint = (currentPosition * 12);  //amt of rotations needed; / (PI * wheelDiameter) (don't need when have conversion factor)
      //frc::SmartDashboard::PutNumber("setpoint", setpoint);
      ShuffleUI::MakeWidget("setpoint", "PLACEHOLDER", setpoint);
      lPID.SetReference(std::copysign(setpoint, totalFeet), rev::CANSparkMax::ControlType::kPosition); //setpoint uses encoder
      rPID.SetReference(std::copysign(setpoint, totalFeet), rev::CANSparkMax::ControlType::kPosition); //everything in abs, so will go backwards
      prevTime = frc::Timer::GetFPGATimestamp().value();
      //frc::SmartDashboard::PutNumber("prevTime", prevTime);
      ShuffleUI::MakeWidget("prevTime", "PLACEHOLDER", prevTime);
    }
  } else {
      while(fabs(currentPosition) < fabs(totalFeet)){
        if(stopAuto) {
          break;
        }
        //frc::SmartDashboard::PutNumber("lEncoder", lEncoder.GetPosition());
        ShuffleUI::MakeWidget("lEncoder", "PLACEHOLDER", lEncoder.GetPosition());
        //frc::SmartDashboard::PutNumber("rEncoder", rEncoder.GetPosition());
        ShuffleUI::MakeWidget("rEncoder", "PLACEHOLDER", rEncoder.GetPosition());
        timeElapsed = frc::Timer::GetFPGATimestamp().value() - prevTime;
        distanceToDeccelerate = (3 * currentVelocity * currentVelocity) / (2 * maxAcc); //change
        if (fabs(distanceToDeccelerate) > fabs(totalFeet - currentPosition)) {
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

        currentPosition += currentVelocity * timeElapsed;
        if(fabs(currentPosition) > fabs(totalFeet)) {
          currentPosition = totalFeet;
        }

        setpoint = (currentPosition * 12);  //amt of rotations needed; / (PI * wheelDiameter) (don't need when have conversion factor)
        //frc::SmartDashboard::PutNumber("setpoint", setpoint);
        ShuffleUI::MakeWidget("setpoint", "PLACEHOLDER", setpoint);
        lPID.SetReference(std::copysign(setpoint, totalFeet), rev::CANSparkMax::ControlType::kPosition); //setpoint uses encoder
        rPID.SetReference(std::copysign(setpoint, totalFeet), rev::CANSparkMax::ControlType::kPosition);
        prevTime = frc::Timer::GetFPGATimestamp().value();
        //frc::SmartDashboard::PutNumber("prevTime", prevTime);
        ShuffleUI::MakeWidget("prevTime", "PLACEHOLDER", prevTime);
    }
  }
  //frc::SmartDashboard::PutBoolean("inPIDDrive", false);
  ShuffleUI::MakeWidget("inPIDDrive", "PLACEHOLDER", false);
  return true;
}

bool DriveBaseModule::PIDTurn(float angle, float radius, bool keepVelocity) { //checks with gyro for accuracy, verify this
  rEncoder.SetPosition(0);
  lEncoder.SetPosition(0);

  //frc::SmartDashboard::PutBoolean("In PIDTurn Function", true);
  ShuffleUI::MakeWidget("In PIDTurn Function", "PLACEHOLDER", true);
  float timeElapsed, distanceToDeccelerate = 0.0; //currentPosition is the set point
  double currentPosition = 0, endpoint = 0; //currentVelocity in class variables
  float prevTime = frc::Timer::GetFPGATimestamp().value();

  endpoint = (fabs(angle) / 360.0) * fabs((radius) + centerToWheel) * (2 * PI); //move fabs to be outside of fabs(radius + centerToWheel)!!!

  bool hasRecalibrated = false;
  gyroOffsetVal = ahrs->GetAngle();
  //fabs(radius + centerToWheel)
  //frc::SmartDashboard::PutNumber("endpoint", endpoint);
  ShuffleUI::MakeWidget("endpoint", "PLACEHOLDER", endpoint);

  if(keepVelocity) {
    while(fabs(currentPosition) < fabs(endpoint)){
       if(stopAuto) {
        break;
      }
      //frc::SmartDashboard::PutNumber("lEncoder", lEncoder.GetPosition());
      ShuffleUI::MakeWidget("lEncoder", "PLACEHOLDER", lEncoder.GetPosition());
      //frc::SmartDashboard::PutNumber("rEncoder", rEncoder.GetPosition());
      ShuffleUI::MakeWidget("rEncoder", "PLACEHOLDER", rEncoder.GetPosition());
      timeElapsed = frc::Timer::GetFPGATimestamp().value() - prevTime;
      currentVelocity += (maxAcc * timeElapsed);
      if (fabs(currentVelocity) > fabs(maxVelocity))
      {
        currentVelocity = maxVelocity;
      }

      currentPosition += currentVelocity * timeElapsed;
      if(fabs(currentPosition) > fabs(endpoint)) {
        currentPosition = endpoint;
      }

         double currAngle = std::copysign(getGyroAngleAuto(), angle); //set before while loop
        //frc::SmartDashboard::PutNumber("curr Angle", currAngle);
        ShuffleUI::MakeWidget("curr Angle", "PLACEHOLDER", currAngle);

        double theoreticalAngle = (currentPosition / endpoint) * angle; //shouldn't use aggregate angle, that's accounted for in endpoint?, or should
        //frc::SmartDashboard::PutNumber("theoretical Angle", theoreticalAngle);
        ShuffleUI::MakeWidget("theoretical Angle", "PLACEHOLDER", theoreticalAngle);


      //this is a test, doesn't work

      if(!hasRecalibrated && (angle * 0.9 < theoreticalAngle))
      if(fabs(currAngle - theoreticalAngle) > 1) { //if offset more than 3 recalculate endpoint
          double adjustment = fabs(theoreticalAngle - currAngle);
          if(currAngle > theoreticalAngle)
            adjustment *= -1;
          //frc::SmartDashboard::PutNumber("adjustment", adjustment); //remove fabs here for agg angle + adjustment, and remove for radius
          ShuffleUI::MakeWidget("adjustment", "PLACEHOLDER", adjustment);
          double newEndpoint = (fabs(angle  + (adjustment)) / 360.0) * fabs(radius + centerToWheel) * (2 * PI); //fabs of angle, same for radius so can turn negative, have an if statement to change dir later
          //angle += adjustment;
          //frc::SmartDashboard::PutNumber("agg angle", angle);
          ShuffleUI::MakeWidget("agg angle", "PLACEHOLDER", angle);
          endpoint = newEndpoint;
          //frc::SmartDashboard::PutNumber("Endpoint Diff", newEndpoint - endpoint);
          ShuffleUI::MakeWidget("Endpoint Diff", "PLACEHOLDER", newEndpoint - endpoint);
          //frc::SmartDashboard::PutNumber("NEW endpoint", newEndpoint);
          ShuffleUI::MakeWidget("NEW endpoint", "PLACEHOLDER", newEndpoint);
          hasRecalibrated = true;
      } 

      double outerSetpoint = (currentPosition * 12); // for now this is ticks (maybe rotations / gearRatio if not then) //change wheel diameter, might not need
      double innerSetpoint = fabs((radius) - centerToWheel)/(fabs((radius) + centerToWheel)) * outerSetpoint; //fabs radius for other things
      
      //frc::SmartDashboard::PutNumber("outerSet", outerSetpoint);
      ShuffleUI::MakeWidget("outerSet", "PLACEHOLDER", outerSetpoint);
      //frc::SmartDashboard::PutNumber("innerSet", innerSetpoint);
      ShuffleUI::MakeWidget("innerSet", "PLACEHOLDER", innerSetpoint);

      int multiplier = 1;
      if(radius < 0) {
        multiplier = -1;
      }
       if(angle > 0) { //angle if reversed outer and inner setpoint reverses, if radius changes change sign of thing,
          lPID.SetReference(multiplier * outerSetpoint, rev::CANSparkMax::ControlType::kPosition);
          rPID.SetReference(multiplier * innerSetpoint, rev::CANSparkMax::ControlType::kPosition);
        } else {
          lPID.SetReference(multiplier * innerSetpoint, rev::CANSparkMax::ControlType::kPosition);
          rPID.SetReference(multiplier * outerSetpoint, rev::CANSparkMax::ControlType::kPosition);
        } //figure this out


      prevTime = frc::Timer::GetFPGATimestamp().value();
      //frc::SmartDashboard::PutNumber("prevTime", prevTime);
      ShuffleUI::MakeWidget("prevTime", "PLACEHOLDER", prevTime);
    }
  } else {

    // double startingAngle = fabs(fmod(gyroSource.ahrs->GetAngle(), 360)); //get starting angle
   // gyroSource.ahrs->SetAngleAdjustment(0);
    
    //double aggregateAngle = angle;

    //frc::SmartDashboard::PutNumber("gyro offset", gyroOffsetVal);
    ShuffleUI::MakeWidget("gyro offset", "PLACEHOLDER", gyroOffsetVal);
    // initGyroAuto();
     while(fabs(currentPosition) < fabs(endpoint)){ 
        if(stopAuto) {
          break;
        }
      //frc::SmartDashboard::PutNumber("lEncoder", lEncoder.GetPosition());
      ShuffleUI::MakeWidget("lEncoder", "PLACEHOLDER", lEncoder.GetPosition());
      //frc::SmartDashboard::PutNumber("rEncoder", rEncoder.GetPosition());
      ShuffleUI::MakeWidget("rEncoder", "PLACEHOLDER", rEncoder.GetPosition());
      timeElapsed = frc::Timer::GetFPGATimestamp().value() - prevTime;
      //should be 2, * Vc^2, check this later
      distanceToDeccelerate = (3 * currentVelocity * currentVelocity) / (2 * maxAcc);
      if (fabs(distanceToDeccelerate) > fabs(endpoint - currentPosition)) {
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
      currentPosition += currentVelocity * timeElapsed;
      
      if(fabs(currentPosition) > fabs(endpoint)) {
        currentPosition = endpoint;
      }

      //dynamic angle adjustment (instead of having a gyro adjust at the end)
      double currAngle = std::copysign(getGyroAngleAuto(), angle); //set before while loop
      //frc::SmartDashboard::PutNumber("curr Angle", currAngle);
      ShuffleUI::MakeWidget("curr Angle", "PLACEHOLDER", currAngle);

      double theoreticalAngle = (currentPosition / endpoint) * angle; //shouldn't use aggregate angle, that's accounted for in endpoint?, or should
      //frc::SmartDashboard::PutNumber("theoretical Angle", theoreticalAngle);
      ShuffleUI::MakeWidget("theoretical Angle", "PLACEHOLDER", theoreticalAngle);


      //this is a test, doesn't work

      if(!hasRecalibrated && (fabs(angle * 0.95) < fabs(theoreticalAngle)))
      if(fabs(currAngle - theoreticalAngle) > 1) { //if offset more than 3 recalculate endpoint
          double adjustment = fabs(theoreticalAngle - currAngle);
          if(currAngle > theoreticalAngle)
            adjustment *= -1;
          //frc::SmartDashboard::PutNumber("adjustment", adjustment); //remove fabs here for agg angle + adjustment, and remove for radius
          ShuffleUI::MakeWidget("adjustment", "PLACEHOLDER", adjustment);
          double newEndpoint = (fabs(angle  + (adjustment)) / 360.0) * fabs(radius + centerToWheel) * (2 * PI); //fabs of angle, same for radius so can turn negative, have an if statement to change dir later
          //angle += adjustment;
          //frc::SmartDashboard::PutNumber("agg angle", angle);
          ShuffleUI::MakeWidget("agg angle", "PLACEHOLDER", angle);
          endpoint = newEndpoint;
          //frc::SmartDashboard::PutNumber("Endpoint Diff", newEndpoint - endpoint);
          ShuffleUI::MakeWidget("Endpoint Diff", "PLACEHOLDER", newEndpoint - endpoint);
          //frc::SmartDashboard::PutNumber("NEW endpoint", newEndpoint);
          ShuffleUI::MakeWidget("NEW endpoint", "PLACEHOLDER", newEndpoint);
          hasRecalibrated = true;
      } 
    
      double outerSetpoint = (currentPosition * 12); // for now this is ticks (maybe rotations / gearRatio if not then) //change wheel diameter, might not need
      double innerSetpoint = ((radius - centerToWheel)/(radius + centerToWheel)) * outerSetpoint;
      
      //frc::SmartDashboard::PutNumber("outerSet", outerSetpoint);
      ShuffleUI::MakeWidget("outerSet", "PLACEHOLDER", outerSetpoint);
      //frc::SmartDashboard::PutNumber("innerSet", innerSetpoint);
      ShuffleUI::MakeWidget("innerSet", "PLACEHOLDER", innerSetpoint);

      int multiplier = 1;
      if(radius < 0) {
        multiplier = -1;
      } //if radius is negative it goes three times the amount it's supposed to go, so divide by 3, works for pos and neg angle though

       if(angle > 0) { //angle if reversed outer and inner setpoint reverses, if radius changes change sign of thing,
          lPID.SetReference(multiplier * outerSetpoint, rev::CANSparkMax::ControlType::kPosition);
          rPID.SetReference(multiplier * innerSetpoint, rev::CANSparkMax::ControlType::kPosition);
        } else {
          lPID.SetReference(multiplier * innerSetpoint, rev::CANSparkMax::ControlType::kPosition);
          rPID.SetReference(multiplier * outerSetpoint, rev::CANSparkMax::ControlType::kPosition);
        } //figure this out


      prevTime = frc::Timer::GetFPGATimestamp().value();
      //frc::SmartDashboard::PutNumber("prevTime", prevTime);
      ShuffleUI::MakeWidget("prevTime", "PLACEHOLDER", prevTime);
    }
  }

  //could call gyroDrive adjustment if angle is off, instead of having the dynamic thingy [takes longer, but super accurate] (did this last year)
  //frc::SmartDashboard::PutBoolean("In PIDTurn Function", false);
  ShuffleUI::MakeWidget("In PIDTurn Function", "PLACEHOLDER", false);
  return true;
}

void DriveBaseModule::initPath() {
  //normally a text file, for testing purposes doing this
  //we also used trig previously and just x, y, now need to incorporate radius
  robPos.x = 0;
  robPos.y = 0;

  pathPoint point1; //example
  point1.x = 2;
  point1.y = 2; 
  straightLinePoints.push_back(point1);

  radiusTurnPoint rpoint1;
  rpoint1.radius = 3; //neg
  rpoint1.angle = 180;
  radiusTurnPoints.push_back(rpoint1);

  // radiusTurnPoint rpoint2;
  // rpoint2.radius = 0; //neg
  // rpoint2.angle = -180;
  // radiusTurnPoints.push_back(rpoint2);


  // pathPoint point2; //example
  // point2.x = 0;
  // point2.y = 10; 
  // straightLinePoints.push_back(point2);
  // //calculate x and y robot manually

  // pathPoint point3; //example
  // point3.x = -5;
  // point3.y = 5; 
  // straightLinePoints.push_back(point3);

  // pathPoint point4; //example
  // point4.x = 0;
  // point4.y = 0; 
  // straightLinePoints.push_back(point4);

  pathOrder.push_back(true);
  pathOrder.push_back(false);
  // pathOrder.push_back(true);
  // pathOrder.push_back(true);
  // pathOrder.push_back(false);
  //pathOrder.push_back(true);
}

void DriveBaseModule::autonomousSequence() {
  initPath();
  int index = 0;
  int lineIndex = 0;
  int curveIndex = 0;
  while(index < pathOrder.size()) {
    if(stopAuto) {
      break;
    }
    if(pathOrder.at(index)) {
      //straight line, doing turn
      pathPoint delta;
      delta.x = (straightLinePoints.at(lineIndex).x - robPos.x);
      delta.y = (straightLinePoints.at(lineIndex).y - robPos.y);

      d = sqrt(pow(delta.x, 2) + pow(delta.y, 2));  

      // pathPoint unitDir;
      // // unitDir.x = delta.x / d;
      // // unitDir.y = delta.y / d;

      // // delta.x = delta.x + unitDir.x * coordOffset; 
      // // delta.y = delta.y + unitDir.y * coordOffset;

     theta = atan2(delta.x, delta.y) * (180/(3.14159265)); 
     //frc::SmartDashboard::PutNumber("theta", theta);
     ShuffleUI::MakeWidget("theta", "PLACEHOLDER", theta);
     //frc::SmartDashboard::PutNumber("d", d);
     ShuffleUI::MakeWidget("d", "PLACEHOLDER", d);

     robPos.x += delta.x;
     robPos.y += delta.y;

    theta = theta - robTheta; //robTheta inited to zero
    robTheta += theta;
     //check with gyro get displacement
     PIDTurn(theta, 0, false); //expiriment with true
     PIDDrive(d, false);
     lineIndex++;

    } else {
      robPos.x += 0; //do math later !!!!
      robPos.y += 0;  //do math later !!!!
      PIDTurn(radiusTurnPoints.at(curveIndex).angle, radiusTurnPoints.at(curveIndex).radius, false);
      curveIndex++;
    }
    index++;
    //frc::SmartDashboard::PutNumber("index", index);
    ShuffleUI::MakeWidget("index", "PLACEHOLDER", index);
  }
}

/*
std::vector<double> DriveBaseModule::getCoords() {
    std::vector<double> temp {current_x, current_y};
    return temp; 
}

void DriveBaseModule::setTarget(double x, double y) {
    target_x = x;
    target_y = y;
  }

void DriveBaseModule::updatePos(double left, double right, double angle) { // Run in a loop, uses gyro, USE CHANGE IN DISTANCE
  double x = position.at(0);
  double y = position.at(1);
  double theta = range360(angle);
  theta = angle * PI / 180; // theta should come in as a radian but output in degrees

  double dcenter = (left + right) / 2;
  double phi = (right - left) / (centerToWheel * 2); // In radians

  double f_theta = theta + phi;
  //f_theta = fmod(f_theta * 180 / wpi::numbers::pi, 360);
  double f_x = x + (dcenter * cos(theta));
  double f_y = y + (dcenter * sin(theta));

  position.at(0) = f_x;
  position.at(1) = f_y;
  //position.at(2) = f_theta;

  }

double DriveBaseModule::getAngleToTarget() {
    double angle;
    double relative_x = fabs(target_x - current_x);
    double relative_y = fabs(target_y - current_y);
    angle = range360(atan2(relative_y, relative_x) * (180 / PI));
    if (current_x > target_x) {
      
      if (current_y > target_y) {
        angle += 180;
        return (range360(current_theta - angle));
        
      } else {
        angle = (180 - angle);
        return (range360(current_theta- angle));
      }
      
    } else {
      if (current_y > target_y) {
        angle = 360 - angle;
        return (range360(current_theta - angle));
      } else {
        return (range360(current_theta - angle));
      }
    }


  }

double DriveBaseModule::getDistanceToTarget() { //Linear Distance NOT 3D
    double xsqr = (current_x - target_x) * (current_x - target_x);
    double ysqr = (current_y - target_y) * (current_y - target_y);
    return (sqrt(xsqr + ysqr));
  }

double DriveBaseModule::range360(double inp) {
    double out;
    if (inp < 0) {
        out = 360 + inp;
        out = fmod(out, 360);
    } else {
        out = fmod(inp, 360);
    }
    return out;
}
*/

void DriveBaseModule::runInit() {
  if (!(initDriveMotor(lMotor, lMotorFollower, lInvert) && initDriveMotor(rMotor, rMotorFollower, rInvert))) {
    //frc::SmartDashboard::PutBoolean("Drive Motor Inits", false);
    ShuffleUI::MakeWidget("Drive Motor Inits", "PLACEHOLDER", false);
  }

  if (!setDriveCurrLimit(motorInitMaxCurrent, motorInitRatedCurrent, motorInitLimitCycles)) {
    //frc::SmartDashboard::PutBoolean("Drive Curr Limits", false);
    ShuffleUI::MakeWidget("Drive Curr Limits", "PLACEHOLDER", false);
  }
  //auto drive PID controllers
  lPID.SetP(PIDProportional);
  lPID.SetI(PIDIntegral);
  lPID.SetD(PIDDerivative);
  lPID.SetIZone(PIDIZone);

  rPID.SetP(PIDProportional);
  rPID.SetI(PIDIntegral);
  rPID.SetD(PIDDerivative);
  rPID.SetIZone(PIDIZone);

  rEncoder.SetPosition(0);
  lEncoder.SetPosition(0);
  rEncoder.SetPositionConversionFactor(1.96); //check if this works! [look at other code, converts rotations to feet, might not need]
  lEncoder.SetPositionConversionFactor(1.96); //gear ratio?
}

void DriveBaseModule::run() {
  runInit();
  bool test = true;
  int counter = 0;
  while(true) { 
    auto nextRun = std::chrono::steady_clock::now() + std::chrono::milliseconds(5); //change milliseconds at telop
    //frc::SmartDashboard::PutNumber("timesRun", ++counter);
    ShuffleUI::MakeWidget("timesRun", "PLACEHOLDER", ++counter);


    //need mutex to stop

    if(state == 'a') { //ik I have access to isAutonomous
      stopAuto = false;
      if(test) {
          //autonomousSequence();
          //PIDDrive(-7, false);
          PIDTurn(-110, 0, false);
          PIDTurn(110, 0, false);

        test = false;
      }
      elev->AutoPeriodic();
      
    } else {
      test = true;
      stopAuto = true;
    }

    if(state == 't') {
      //perioidic routines
      gyroDriving();

      //TRP
      if (driverStick->GetRawButton(1)) {
   //     position.at(0) = 0;
   //     position.at(1) = 0;
        ahrs->Reset();
      }
      double left = lEncoder.GetPosition();
      double right = rEncoder.GetPosition();
      //frc::SmartDashboard::PutNumber("left", left);
      ShuffleUI::MakeWidget("left", "PLACEHOLDER", left);
      //frc::SmartDashboard::PutNumber("right", right);
      ShuffleUI::MakeWidget("right", "PLACEHOLDER", right);
   //   updatePos(left - last_Lencoder, right - last_Rencoder, ahrs->GetAngle());
      //frc::SmartDashboard::PutNumber("X", position.at(0));
   // ShuffleUI::MakeWidget("X", "PLACEHOLDER", position.at(0));
      //frc::SmartDashboard::PutNumber("Y", position.at(1));
   // ShuffleUI::MakeWidget("Y", "PLACEHOLDER", position.at(1));
      
   //   last_Lencoder = left;
   //   last_Rencoder = right;
      //End TRP
      //honestly let's move to xbox joystick maybe
      //elev->TeleopPeriodic(driverStick->GetLeftTriggerAxis(), driverStick->GetRightTriggerAxis()); 
      test = true;
      stopAuto = true;
    }

    if(state == 'u') { //tuning auto PID's
     PIDTuning();
    }


    if(state == 'd') { //tuning drive PID's
      driveBaseTuning();
    }

    //ADJUST NEXT RUN MAYBE
    std::this_thread::sleep_until(nextRun); //need this here so thread runs every 5 ms, might be faster than PID controller look into it
  }
}





