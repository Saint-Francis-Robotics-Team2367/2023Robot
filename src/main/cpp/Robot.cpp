// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

//#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "ScaraArmModule.h"
#include "DriveBaseModule.h"
#include "ElevatorModule.h"
#include "Paths.h"
#include <frc/XboxController.h>

frc::XboxController* ctr = new frc::XboxController(0);
frc::XboxController* ctr2 = new frc::XboxController(1);

ScaraArmModule scaraArm(ctr2);
DriveBaseModule drive;
ElevatorModule elevator(ctr);

autoPath path[5] = {
    autoPath(autoPathType::straight),
    autoPath(autoPathType::turn),
    autoPath(autoPathType::elev), 
    autoPath(autoPathType::straight),
    autoPath(autoPathType::arm),
  };

//Limelight ll; 

//cpr number pulses (4096) per rev, 70 to 1 / 360

void Robot::RobotInit()
{
  drive.driveThread.detach(); 
  scaraArm.scaraArmThread.detach();
  elevator.elevatorThread.detach();
}

void Robot::RobotPeriodic() {
  //Get TargetPose Robot Space:
  // std::vector<double> targetPose = ll.getTargetPoseRobotSpace();
  // Limelight::Point targetXY = ll.getTargetXY(targetPose.at(0) * 39.37, targetPose.at(2) * 39.37, targetPose.at(4), Limelight::bottomLeftPole); // X, Y, yaw, poleID
  // frc::SmartDashboard::PutNumber("TapeX", targetXY.x);
  // frc::SmartDashboard::PutNumber("TapeY", targetXY.y);
  // frc::SmartDashboard::PutNumber("Detected?", ll.getTargetDetected());
}

void Robot::AutonomousInit() {
  scaraArm.state = 'a';
  drive.state = 'a';
  elevator.state = 'a';

  autoPath a(autoPathType::straight); 
  a.register_straight(1); 

  autoPath b(autoPathType::turn); 
  b.register_turn(90, 2);

  autoPath c(autoPathType::elev); 
  c.register_elev(25, true); 

  autoPath d(autoPathType::straight); 
  d.register_straight(1);

  autoPath e(autoPathType::arm); 
  e.register_arm(20, 20); 

  // assigning path points  
  path[0] = a; 
  path[1] = b; 
  path[2] = c; 
  path[3] = d;
  path[4] = e; 

  // drive.initPath(); 
}

void Robot::AutonomousPeriodic() {
  bool testing = true; 
  int index = 0; 

  drive.stopAuto = false;
  elevator.stopAuto = false; 
  scaraArm.stopAuto = false; 

  int numSteps = 5; // CHANGE DEPENDING ON LENGTH OF PATH LIST! 
  float angle, radius; 

  if(testing) {
    while(index < numSteps) {
      //frc::SmartDashboard::PutBoolean("straight", path[index].straight); 
      switch(path[index].action){
        case 's': // driving straight 
          if(drive.stopAuto) {
            break;
          }
          frc::SmartDashboard::PutNumber("dis", path[index].dis);
          drive.PIDDrive(path[index].dis, path[index].keepVelocity);
          break; 

        case 't': // turn
          if(drive.stopAuto) {
            break;
          }
          // retrieving angle and radius of turn 
          angle = path[index].angle; 
          radius = path[index].radius; 
          frc::SmartDashboard::PutNumber("angle", angle); 
          frc::SmartDashboard::PutNumber("radius", radius);

          if (angle > 0){ // robot starts at 180 deg for right turns 
            angle = -(angle - 180);
          }
        
          drive.PIDTurn(angle, radius, path[index].keepVelocity);
          break; 

        case 'e': // elevator
          if(elevator.stopAuto) {
            break;
          }
          elevator.setPos(path[index].setpoint, path[index].motionProfiling); 
          break;

        case 'a': // arm 
          if(scaraArm.stopAuto) {
            break;
          }
          scaraArm.movetoXY(path[index].arm_x, path[index].arm_y); 
          break; 

        default: 
          break; 
      }



      index++; 
    }



    testing = false;
  }
  else {
    testing = true;
    drive.stopAuto = true;
    elevator.stopAuto = true;
    scaraArm.stopAuto = true;
  }

}

void Robot::TeleopInit() {
  scaraArm.state = 't';
  drive.state = 't';
  elevator.state = 't';
}

void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {
}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif