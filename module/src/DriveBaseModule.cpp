#include "DriveBaseModule.h"
#include <iostream>
#include "ShuffleUI.h"

DriveBaseModule::DriveBaseModule() {
    ahrs = new AHRS(frc::SerialPort::kMXP);
    driveThread = std::thread(&DriveBaseModule::run, this); //initializing thread so can detach in robot init
}

bool DriveBaseModule::initDriveMotor(rev::CANSparkMax* motor, rev::CANSparkMax* follower, bool invert) {
  motor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  motor->SetInverted(invert);
  follower->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
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

//need to TEST SKIM CONSTANT
void DriveBaseModule::arcadeDrive(double xSpeedi, double zRotationi) {
    if (fabs(xSpeedi) < xDeadband)
        xSpeedi = 0;

    if (fabs(zRotationi) < yDeadband)
        zRotationi = 0;

    double xSpeed = std::copysign(pow(fabs(xSpeedi), 2.5), xSpeedi);
    double zRotation = std::copysign(pow(fabs(zRotationi), 2.5), zRotationi);

    LimitRate(xSpeed, zRotation);

    double leftMotorOutput = xSpeed + zRotation;
    double rightMotorOutput = xSpeed - zRotation; //removed turnSense need test

    if (leftMotorOutput != 0)
        leftMotorOutput = std::copysign((1/(1-xDeadband)) * fabs(leftMotorOutput) - (xDeadband/(1/xDeadband)), leftMotorOutput);
        
    if (rightMotorOutput != 0)
        rightMotorOutput = std::copysign((1/(1-yDeadband)) * fabs(rightMotorOutput) - (yDeadband/(1/yDeadband)), rightMotorOutput);

    leftMotorOutput = std::clamp(leftMotorOutput, -1.0, 1.0);
    rightMotorOutput = std::clamp(rightMotorOutput, -1.0, 1.0);
    lMotor->Set(leftMotorOutput);
    rMotor->Set(rightMotorOutput); //try SetVoltage
}

void DriveBaseModule::gyroDriving() {
  float rightStickOutput = driverStick->GetRawAxis(4);
  frc::SmartDashboard::PutNumber("right stick output", rightStickOutput);
  float calculation = rightStickPID->Calculate(ahrs->GetRate()/150, rightStickOutput); //add skim
  arcadeDrive(driverStick->GetRawAxis(1) * (-1), calculation);
  ShuffleUI::MakeWidget("output", tab, calculation);
  ShuffleUI::MakeWidget("gyro", tab, ahrs->GetRate());

}

double DriveBaseModule::getTilt() {
  double pitch = ahrs->GetPitch();
	double roll = ahrs->GetRoll();
    if((pitch + roll)>= 0){
        return std::sqrt(pitch*pitch + roll*roll);
    } else {
        return -std::sqrt(pitch*pitch + roll*roll);
    }
}

void DriveBaseModule::autoBalance() 
{
  //make autoBalance init()
  // 17 in to travel the shorter part of the ramp
  // 26 inches til midway of the ramp (~39 inches to travel so that 1/4 away from edge of ramp)
  //tilt is negative on first start
  double tilt = getTilt(); //you also might be able to just use roll() or just use pitch(), but both work
  frc::SmartDashboard::PutNumber("Tilt", tilt);
  //frc::SmartDashboard::PutNumber("stage", stateCounter);
  bool attemptedClimbOnce = false;
  
  switch (balanceState) {
    frc::SmartDashboard::PutNumber("state", balanceState);
    //make init for this...
    case autoBalanceStages::align : // align
        // if(fabs(offsetYaw - ahrs->GetAngle()) > 2) { // test value for yaw (angle)
        //   //PIDTurn(offsetYaw - ahrs->GetAngle(), 0, true); //account for negatives later ?
        // }
        lEncoder.SetPosition(0);
        currEncoderPos = lEncoder.GetPosition(); //setting up currEncoderPos for initialClimb
        balanceState = autoBalanceStages::initialClimb;
        frc::SmartDashboard::PutBoolean("aligned", true);
        break;
    

    case autoBalanceStages::initialClimb : // initial climb

      frc::SmartDashboard::PutBoolean("moving to target", true);
      frc::SmartDashboard::PutNumber("currEncoderPos", currEncoderPos);
      if(!(currEncoderPos < -30))  
      {
        lPID.SetReference(currEncoderPos, rev::CANSparkMax::ControlType::kPosition);
        rPID.SetReference(currEncoderPos, rev::CANSparkMax::ControlType::kPosition);
        currEncoderPos -= rampSpeed;
      }
      
        // implement 2nd try to climb
      if(tilt > maxRampAngle) 
      { // do we want to add deg of freedom?
        currEncoderPos = lEncoder.GetPosition();
        balanceState = autoBalanceStages::balance;
        frc::SmartDashboard::PutBoolean("initial climb", true);
      }
      break;
    
    case autoBalanceStages::balance :
      frc::SmartDashboard::PutBoolean("balance", true);
      // do I need some timer to wait for next command?
      frc::SmartDashboard::PutNumber("currEncoderPos", currEncoderPos);

      // initially robot is driving backwards so this value should be negative for the first half of the ramp, then if it goes over drive forward
      if (tilt < maxRampAngle - 2)
      {                                                                              // two degrees of freedom to just stop
        lPID.SetReference(currEncoderPos, rev::CANSparkMax::ControlType::kPosition); // setpoint uses encoder (tilt - 0 = error, you don't need the 0)
        rPID.SetReference(currEncoderPos, rev::CANSparkMax::ControlType::kPosition); // everything in abs, so will go backwards
        } 
        else 
        {
            if(tilt > 0)
            {                      // tilt going up initially is positive
              currEncoderPos -= rampSpeed; //temporary incremenet amount
            }
            else 
            {
              currEncoderPos += rampSpeed;
            }

          lPID.SetReference(currEncoderPos, rev::CANSparkMax::ControlType::kPosition);
          rPID.SetReference(currEncoderPos, rev::CANSparkMax::ControlType::kPosition);
        }
        // //Change this later
        // lEncoder.SetPosition(0); //this is to continue the loop to continuously update the error
        // rEncoder.SetPosition(0);
        frc::SmartDashboard::PutBoolean("balancing", true);
        
        // currEncoderPos += tilt;
        // if(currEncoderPos < 0) {
        //   // stateCounter++; //if it somehow drives backwards all the way down the ramp stop the robot
        //   lMotor->StopMotor();
        //   rMotor->StopMotor();
        // }
        break;
    }
}

// void DriveBaseModule::autoBalance() {
//   // 17 in to travel the shorter part of the ramp
//   // 26 inches til midway of the ramp (~39 inches to travel so that 1/4 away from edge of ramp)
//   //tilt is negative on first start
//   double tilt = getTilt(); //you also might be able to just use roll() or just use pitch(), but both work
//   frc::SmartDashboard::PutNumber("Tilt", tilt);
//   frc::SmartDashboard::PutNumber("stage", stateCounter);
  

//   if(stateCounter==0) {
//     if(!hasStarted) {
//       offsetTilt = tilt; //the original balanced thing
//       offsetYaw = ahrs->GetAngle(); //gets the angle of the actual robot (yaw)
//       // offsetYaw = ahrs->ZeroYaw();
//       hasStarted = true;
//     }
//       if(fabs(getTilt()) < 11) { //go forward until we are on a known location at charge

//         if(!firstRun) {
//           // PIDDrive(40, false); //change this to SetReference later maybe
//           lPID.SetReference(-24, rev::CANSparkMax::ControlType::kPosition);
//           rPID.SetReference(-24, rev::CANSparkMax::ControlType::kPosition);

//           frc::SmartDashboard::PutBoolean("forward", true);
//           firstRun = true;
//         }

//         if(firstRun && !secondRun) {
//           // align with charge station
//           if(fabs(offsetYaw - ahrs->GetAngle()) > 2) { // test value for yaw (angle)
//             PIDTurn(offsetYaw - ahrs->GetAngle(), 0, true); //account for negatives later ?
//           }
               
//           // PIDDrive(40, false);
//           // how to make negative
//           lPID.SetReference(-24, rev::CANSparkMax::ControlType::kPosition);
//           rPID.SetReference(-24, rev::CANSparkMax::ControlType::kPosition);
//           secondRun = true;
//         }
        
//       } else {
//         stateCounter++;
//         frc::SmartDashboard::PutBoolean("forward", false);
//   } 

//   if(stateCounter == 1) { //at known position set encoder values to 0
//       // lMotor->StopMotor(); //bridge between arcade drive and PID's
//       // rMotor->StopMotor();
//       lEncoder.SetPosition(0);
//       rEncoder.SetPosition(0); 
//       currEncoderPos = lEncoder.GetPosition();
//       stateCounter++;
//   }

//   if(stateCounter == 2) {
//       currEncoderPos += tilt;
//       if(currEncoderPos < 0) {
//         // stateCounter++; //if it somehow drives backwards all the way down the ramp stop the robot
//         lMotor->StopMotor();
//         rMotor->StopMotor();
//       }

//       //do I need some timer to wait for next command?
//       frc::SmartDashboard::PutNumber("currEncoderPos", currEncoderPos);
//       // initially robot is driving backwards so this value should be negative for the first half of the ramp, then if it goes over drive forward
//       lPID.SetReference(tilt - 0, rev::CANSparkMax::ControlType::kPosition); //setpoint uses encoder (tilt - 0 = error, you don't need the 0)
//       rPID.SetReference(tilt - 0, rev::CANSparkMax::ControlType::kPosition); //everything in abs, so will go backwards

//       //Change this later
//       lEncoder.SetPosition(0); //this is to continue the loop to continuously update the error
//       rEncoder.SetPosition(0);

//   }
// }
// }

void DriveBaseModule::PIDTuning() {
  //double prevTime = frc::Timer::GetFPGATimestamp().value();
  double currentLeftLead = lMotor->GetOutputCurrent();
  double currentRightLead = rMotor->GetOutputCurrent();

  double voltageOverall = lMotor->GetBusVoltage() + rMotor->GetBusVoltage();
  //do getBusVoltage (returns voltage fed into motor controller)


  ShuffleUI::MakeWidget("Total Current", tab, currentLeftLead+currentRightLead);
  ShuffleUI::MakeWidget("Total Voltage", tab, voltageOverall);

  //Making it so you can manually set m_p and positionTotal: m_p is essential with PID, change by an order of magnitude to start run
  double m_P = ShuffleUI::GetDouble("Pd", tab, 1);
  //bool isNegative;
  lPID.SetP(m_P);
  rPID.SetP(m_P);
  ShuffleUI::MakeWidget("Pd", tab, m_P);

  double m_D = ShuffleUI::GetDouble("D Value", tab, 0);
  //bool isNegative;
  lPID.SetD(m_D);
  rPID.SetD(m_D);
  ShuffleUI::MakeWidget("D Value", tab, m_D);

  double m_I = ShuffleUI::GetDouble("I Value", tab, 0);
  //bool isNegative;
  lPID.SetI(m_I);
  rPID.SetI(m_I);
  ShuffleUI::MakeWidget("I Value", tab, m_I);

 double I_Zone = ShuffleUI::GetDouble("I_Zone", tab, 0);
  //bool isNegative;
  lPID.SetIZone(I_Zone);
  rPID.SetIZone(I_Zone);
  ShuffleUI::MakeWidget("I_Zone", tab, I_Zone);

  lPID.SetIZone(I_Zone);

  double waitTime = ShuffleUI::GetDouble("waitTime", tab, 4);
  ShuffleUI::MakeWidget("waitTime", tab, waitTime);

  double currTime = frc::Timer::GetFPGATimestamp().value();
  ShuffleUI::MakeWidget("currTime", tab, currTime);
  ShuffleUI::MakeWidget("Setpoint", tab, delta);
  if(currTime > tuningPrevTime + waitTime) {
   
      lPID.SetReference(delta, rev::ControlType::kPosition);
      rPID.SetReference(delta, rev::ControlType::kPosition);
      delta = delta * -1.0;
 
      ShuffleUI::MakeWidget("Right Encoder", tab, rEncoder.GetPosition());
      ShuffleUI::MakeWidget("Left Encoder", tab, lEncoder.GetPosition());

      tuningPrevTime = frc::Timer::GetFPGATimestamp().value();
      ShuffleUI::MakeWidget("prev time", tab, tuningPrevTime);
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
  ShuffleUI::MakeWidget("inPIDDrive", tab, true);

  if(keepVelocity) {
    while(fabs(currentPosition) < fabs(totalFeet)) {
      if(stopAuto) {
        break;
      }
      ShuffleUI::MakeWidget("lEncoder", tab, lEncoder.GetPosition());
      ShuffleUI::MakeWidget("rEncoder", tab, rEncoder.GetPosition());
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
      ShuffleUI::MakeWidget("setpoint", tab, setpoint);
      lPID.SetReference(std::copysign(setpoint, totalFeet), rev::CANSparkMax::ControlType::kPosition); //setpoint uses encoder
      rPID.SetReference(std::copysign(setpoint, totalFeet), rev::CANSparkMax::ControlType::kPosition); //everything in abs, so will go backwards
      prevTime = frc::Timer::GetFPGATimestamp().value();
      ShuffleUI::MakeWidget("prevTime", tab, prevTime);
    }
  } else {
      while(fabs(currentPosition) < fabs(totalFeet)){
        if(stopAuto) {
          break;
        }
        ShuffleUI::MakeWidget("lEncoder", tab, lEncoder.GetPosition());
        ShuffleUI::MakeWidget("rEncoder", tab, rEncoder.GetPosition());
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
        ShuffleUI::MakeWidget("setpoint", tab, setpoint);
        lPID.SetReference(std::copysign(setpoint, totalFeet), rev::CANSparkMax::ControlType::kPosition); //setpoint uses encoder
        rPID.SetReference(std::copysign(setpoint, totalFeet), rev::CANSparkMax::ControlType::kPosition);
        prevTime = frc::Timer::GetFPGATimestamp().value();
        ShuffleUI::MakeWidget("prevTime", tab, prevTime);
    }
  }
  ShuffleUI::MakeWidget("inPIDDrive", tab, false);
  return true;
}

bool DriveBaseModule::PIDTurn(float angle, float radius, bool keepVelocity) { //checks with gyro for accuracy, verify this
  rEncoder.SetPosition(0);
  lEncoder.SetPosition(0);

  ShuffleUI::MakeWidget("In PIDTurn Function", tab, true);
  float timeElapsed, distanceToDeccelerate = 0.0; //currentPosition is the set point
  double currentPosition = 0, endpoint = 0; //currentVelocity in class variables
  float prevTime = frc::Timer::GetFPGATimestamp().value();

  endpoint = (fabs(angle) / 360.0) * fabs((radius) + centerToWheel) * (2 * PI); //move fabs to be outside of fabs(radius + centerToWheel)!!!

  bool hasRecalibrated = false;
  gyroOffsetVal = ahrs->GetAngle();
  //fabs(radius + centerToWheel)
  ShuffleUI::MakeWidget("endpoint", tab, endpoint);

  if(keepVelocity) {
    while(fabs(currentPosition) < fabs(endpoint)){
       if(stopAuto) {
        break;
      }
      ShuffleUI::MakeWidget("lEncoder", tab, lEncoder.GetPosition());
      ShuffleUI::MakeWidget("rEncoder", tab, rEncoder.GetPosition());
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
        ShuffleUI::MakeWidget("curr Angle", tab, currAngle);

        double theoreticalAngle = (currentPosition / endpoint) * angle; //shouldn't use aggregate angle, that's accounted for in endpoint?, or should
        ShuffleUI::MakeWidget("theoretical Angle", tab, theoreticalAngle);


      //this is a test, doesn't work

      if(!hasRecalibrated && (angle * 0.9 < theoreticalAngle))
      if(fabs(currAngle - theoreticalAngle) > 1) { //if offset more than 3 recalculate endpoint
          double adjustment = fabs(theoreticalAngle - currAngle);
          if(currAngle > theoreticalAngle)
            adjustment *= -1;
          //frc::SmartDashboard::PutNumber("adjustment", adjustment); //remove fabs here for agg angle + adjustment, and remove for radius
          double newEndpoint = (fabs(angle  + (adjustment)) / 360.0) * fabs(radius + centerToWheel) * (2 * PI); //fabs of angle, same for radius so can turn negative, have an if statement to change dir later
          //angle += adjustment;
          ShuffleUI::MakeWidget("agg angle", tab, angle);
          endpoint = newEndpoint;
          ShuffleUI::MakeWidget("Endpoint Diff", tab, newEndpoint - endpoint);
          ShuffleUI::MakeWidget("NEW endpoint", tab, newEndpoint);
          hasRecalibrated = true;
      } 

      double outerSetpoint = (currentPosition * 12); // for now this is ticks (maybe rotations / gearRatio if not then) //change wheel diameter, might not need
      double innerSetpoint = fabs((radius) - centerToWheel)/(fabs((radius) + centerToWheel)) * outerSetpoint; //fabs radius for other things
      
      ShuffleUI::MakeWidget("outerSet", tab, outerSetpoint);
      ShuffleUI::MakeWidget("innerSet", tab, innerSetpoint);

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
      ShuffleUI::MakeWidget("prevTime", tab, prevTime);
    }
  } else {

    // double startingAngle = fabs(fmod(gyroSource.ahrs->GetAngle(), 360)); //get starting angle
   // gyroSource.ahrs->SetAngleAdjustment(0);
    
    //double aggregateAngle = angle;

    ShuffleUI::MakeWidget("gyro offset", tab, gyroOffsetVal);
    // initGyroAuto();
     while(fabs(currentPosition) < fabs(endpoint)){ 
        if(stopAuto) {
          break;
        }
      ShuffleUI::MakeWidget("lEncoder", tab, lEncoder.GetPosition());
      ShuffleUI::MakeWidget("rEncoder", tab, rEncoder.GetPosition());
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
      ShuffleUI::MakeWidget("curr Angle", tab, currAngle);

      double theoreticalAngle = (currentPosition / endpoint) * angle; //shouldn't use aggregate angle, that's accounted for in endpoint?, or should
      ShuffleUI::MakeWidget("theoretical Angle", tab, theoreticalAngle);


      //this is a test, doesn't work

      if(!hasRecalibrated && (fabs(angle * 0.95) < fabs(theoreticalAngle)))
      if(fabs(currAngle - theoreticalAngle) > 1) { //if offset more than 3 recalculate endpoint
          double adjustment = fabs(theoreticalAngle - currAngle);
          if(currAngle > theoreticalAngle)
            adjustment *= -1;
          //frc::SmartDashboard::PutNumber("adjustment", adjustment); //remove fabs here for agg angle + adjustment, and remove for radius
          double newEndpoint = (fabs(angle  + (adjustment)) / 360.0) * fabs(radius + centerToWheel) * (2 * PI); //fabs of angle, same for radius so can turn negative, have an if statement to change dir later
          //angle += adjustment;
          ShuffleUI::MakeWidget("agg angle", tab, angle);
          endpoint = newEndpoint;
          ShuffleUI::MakeWidget("Endpoint Diff", tab, newEndpoint - endpoint);
          ShuffleUI::MakeWidget("NEW endpoint", tab, newEndpoint);
          hasRecalibrated = true;
      } 
    
      double outerSetpoint = (currentPosition * 12); // for now this is ticks (maybe rotations / gearRatio if not then) //change wheel diameter, might not need
      double innerSetpoint = ((radius - centerToWheel)/(radius + centerToWheel)) * outerSetpoint;
      
      ShuffleUI::MakeWidget("outerSet", tab, outerSetpoint);
      ShuffleUI::MakeWidget("innerSet", tab, innerSetpoint);

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
      ShuffleUI::MakeWidget("prevTime", tab, prevTime);
    }
  }

  //could call gyroDrive adjustment if angle is off, instead of having the dynamic thingy [takes longer, but super accurate] (did this last year)
  ShuffleUI::MakeWidget("In PIDTurn Function", tab, false);
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

void DriveBaseModule::autoDrive(float totalFeetSent, bool keepVelocitySent) {
  totalFeet = totalFeetSent;
  keepVelocityDrive = keepVelocitySent;
  isRunningAutoDrive = true; //do it here
}

void DriveBaseModule::autoTurn(float angle, float radius, bool keepVelocity) {
  this->angle = angle;
  this->radius = radius;
  this->keepVelocityTurn = keepVelocityTurn;
  isRunningAutoTurn = true; //do it here
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
      ShuffleUI::MakeWidget("theta", tab, theta);
      ShuffleUI::MakeWidget("d", tab, d);

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
    ShuffleUI::MakeWidget("index", tab, index);
  }
}

void DriveBaseModule::runInit() {
  if (!(initDriveMotor(lMotor, lMotorFollower, lInvert) && initDriveMotor(rMotor, rMotorFollower, rInvert))) {
    ShuffleUI::MakeWidget("Drive Motor Inits", tab, false);
  }

  if (!setDriveCurrLimit(motorInitMaxCurrent, motorInitRatedCurrent, motorInitLimitCycles)) {
    ShuffleUI::MakeWidget("Drive Curr Limits", tab, false);
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

  offsetTilt = getTilt(); //the original balanced thing
  offsetYaw = ahrs->GetAngle(); //gets the angle of the actual robot (yaw)
}

void DriveBaseModule::run() {
  runInit();
  // arm->ArmInit();
  bool test = true;
  int counter = 0;

  while(true) { 
    double tilt = getTilt(); 
    frc::SmartDashboard::PutNumber("Tilt", tilt);
    auto nextRun = std::chrono::steady_clock::now() + std::chrono::milliseconds(5); //change milliseconds at telop
    ShuffleUI::MakeWidget("timesRun", tab, ++counter);
    
    //frc::SmartDashboard::PutNumber("drivebase y", driverStick->GetRawAxis(1));

    //need mutex to stop

 if(state == 'a') {
    frc::SmartDashboard::PutBoolean("In auto Drive base", true);
    autoBalance();
      // if(isRunningAutoTurn) { //default false
      //   isRunningAutoTurn = false;
      //   isFinished = PIDTurn(angle, radius, keepVelocityTurn);
      //   frc::SmartDashboard::PutBoolean("isRunningAutoTurn", isRunningAutoTurn);
      // }

      // if(isRunningAutoDrive) {
      //   isRunningAutoDrive = false;
      //   isFinished = PIDDrive(totalFeet, keepVelocityDrive);
      //   // PIDDrive(totalFeet, keepVelocityDrive);
      //   // isFinished = true;
      //   frc::SmartDashboard::PutBoolean("isFinished", isFinished);
      //   frc::SmartDashboard::PutBoolean("isRunningAutoDrive", isRunningAutoDrive);

      // }

      // if(balancing) {
      //   autoBalance();
      // }
 }

    if(state == 't') {
      //perioidic routines
      gyroDriving();
      
      ShuffleUI::MakeWidget("joystick", tab, driverStick->GetRawAxis(1));
      //honestly let's move to xbox joystick maybe
      //elev->TeleopPeriodic(driverStick->GetLeftTriggerAxis(), driverStick->GetRightTriggerAxis());
      ShuffleUI::MakeWidget("lcheck", tab, lMotor->GetIdleMode() == rev::CANSparkMax::IdleMode::kBrake); 
      ShuffleUI::MakeWidget("rcheck", tab, rMotor->GetIdleMode() == rev::CANSparkMax::IdleMode::kBrake); 
      
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






