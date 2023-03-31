// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

//#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "ScaraArmModule.h"
#include "DriveBaseModule.h"
#include "ElevatorModule.h"
#include <frc/XboxController.h>
#include "Paths.h"

DriveBaseModule drive;

frc::XboxController* ctr = new frc::XboxController(0);
frc::XboxController* ctr2 = new frc::XboxController(1);

rev::CANSparkMax *leftStar = new rev::CANSparkMax(leftStarID, rev::CANSparkMax::MotorType::kBrushed);
rev::CANSparkMax *rightStar = new rev::CANSparkMax(rightStarID, rev::CANSparkMax::MotorType::kBrushed);

ScaraArmModule scaraArm(ctr, ctr2);

ElevatorModule elevator(ctr, ctr2);

autoPath path[1] = {
    autoPath(autoPathType::elev),
    //autoPath(autoPathType::arm),
    // autoPath(autoPathType::turn), 
    //autoPath(autoPathType::straight),
    //autoPath(autoPathType::arm),
  };



//cpr number pulses (4096) per rev, 70 to 1 / 360

void Robot::RobotInit()
{
  drive.driveThread.detach(); 
  scaraArm.scaraArmThread.detach();
  elevator.elevatorThread.detach();
}

void Robot::RobotPeriodic() {

}

void Robot::AutonomousInit() {
  scaraArm.state = 'a';
  drive.state = 'a';
  elevator.state = 'a';
  index = 0;

  // frc::SendableChooser<std::string> chooser;
  // const std::string autoDefault = "Default";
  // const std::string autoCustom = "Path 1"; // define more variables if there are more custom paths 
  // std::string selected; 

  autoPath a(autoPathType::elev); 
  a.register_elev(30.0); 
  
  // autoPath x(autoPathType::arm);
  // x.register_arm(10, 10);


  // autoPath b(autoPathType::arm); 
  // b.register_turn(90, 2);

  // autoPath c(autoPathType::straight); 
  // c.register_elev(25); 

  // autoPath d(autoPathType::straight); 
  // d.register_straight(5);
  // elevator.isRunningAuto = true;
  // scaraArm.autoStart = true;
  // autoPath e(autoPathType::arm); 
  // e.register_arm(20, 20); 


  // // custom path 1 points:
  // autoPath a1(autoPathType::straight); 
  // a1.register_straight(2); 

  // autoPath b1(autoPathType::turn); 
  // b1.register_turn(90, 2);

  // autoPath c1(autoPathType::turn); 
  // c1.register_turn(-45, 1);

  // autoPath d1(autoPathType::turn); 
  // d1.register_turn(360, 0);
  

  // selected = chooser.GetSelected(); 

  // assigning path points  
  // path[0] = a; 
  // path[1] = x; 
  // path[2] = d; 
  // path[3] = d;
  // path[4] = e; 

  path[0] = a;

  // if (selected == autoCustom){ // if custom path 1 is selected 
  //   autoPath path[4] = {
  //   autoPath(autoPathType::straight),
  //   autoPath(autoPathType::turn),
  //   autoPath(autoPathType::turn),
  //   autoPath(autoPathType::turn),
  //   };

  //   path[0] = a1; 
  //   path[1] = b1; 
  //   path[2] = c1; 
  // //   path[3] = d1;
  // leftStar->Set(-1);
  // rightStar->Set(-1);
  // timestamp = frc::Timer::GetFPGATimestamp().value();
  
  }
  
void Robot::AutonomousPeriodic() {
  // if(frc::Timer::GetFPGATimestamp().value() - timestamp < 5) {
  //   leftStar->Set(-1);
  //   rightStar->Set(-1);
  // }


  int numSteps = 1; // CHANGE DEPENDING ON LENGTH OF PATH LIST! 
  float angle, radius; 

  if(index < numSteps){
     switch(path[index].action){
       case 's': // driving straight
      
        if(!isStage) { // if its not currently driving 
          drive.autoDrive(path[index].dis, path[index].keepVelocity);
          isStage = true; 
          frc::SmartDashboard::PutBoolean("in stage drive", isStage); 
          frc::SmartDashboard::PutBoolean("is finished", drive.isFinished);
          frc::SmartDashboard::PutNumber("dis", path[index].dis);
        } 
        
        if (drive.isFinished){ // once drive is finished 
          isStage = false; 
          drive.isFinished = false; 
        }
        break; 

      case 't': // turn
        if (!isStage){
          angle = path[index].angle; 
          radius = path[index].radius; 
          frc::SmartDashboard::PutNumber("angle", angle); 
          frc::SmartDashboard::PutNumber("radius", radius);

          if (angle > 0){ // robot starts at 180 deg for right turns 
            angle = -(angle - 180);
          }

          drive.autoTurn(angle, radius, path[index].keepVelocity);
          isStage = true; 
        }
        
        if (drive.isFinished ){
          isStage = false; 
          drive.isFinished = false; 
        }
        
        break; 

      case 'e': // elevator
        if (!isStage){
          elevator.autoSet(path[index].setpoint);
          frc::SmartDashboard::PutNumber("setpoint  in robot.cpp", path[index].setpoint);
          //elevator.autoAmount = path[index].setpoint; 
          isStage = true; 
          frc::SmartDashboard::PutBoolean("in stage elev", isStage); 
        }

        if (elevator.isFinished){
          isStage = false; 
          elevator.isFinished = false; 
        }
        
        break;

      case 'a': 
        if(!isStage) {
          scaraArm.autoStart = true;
          isStage = true;
          frc::SmartDashboard::PutBoolean("in stage arm", isStage); 
        }

        if(scaraArm.isFinished) {
          isStage = false;
          scaraArm.isFinished = false;
        }
        
        //scaraArm.movetoXY(path[index].arm_x, path[index].arm_y); 
        break; 

      default: 
        break; 
    }
    
    if (!isStage){
        index++;
    }

     
  }
}


void Robot::TeleopInit() {
  scaraArm.state = 't';
  drive.state = 't';
  elevator.state = 't';
  frc::SmartDashboard::PutNumber("scale", 1.0);
}

void Robot::TeleopPeriodic() {
  
  float scale = frc::SmartDashboard::GetNumber("scale", 0.0);

  if(ctr->GetRawButton(5))
  {
    const float forward = 0.5F * scale;
    leftStar->Set(forward);
    rightStar->Set(forward);
  } 
  else if (ctr->GetRawButton(6))
  {
    const float reverse = -0.5F * scale;
    leftStar->Set(reverse);
    rightStar->Set(reverse);
  }
  else
  {
    leftStar->StopMotor();
    rightStar->StopMotor();
  }
  
   scale = frc::SmartDashboard::PutNumber("scale", scale);



}

void Robot::DisabledInit() {
  scaraArm.state = 'd';
  // drive.state = 'd';
  // elev.state = 'd';
}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif