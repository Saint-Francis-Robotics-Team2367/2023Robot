#include "Macros.h"
#include <vector>
#include <math.h> 
#include <rev/CANSparkMax.h>
#include <frc/Joystick.h>
#include "ElevatorModule.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include "AHRS.h"
#include <frc/controller/PIDController.h>
#include <thread>
#include <chrono>
#include<mutex>
#include <atomic>

#define driverStickPort 0
#define operatorStickPort 1

#define PIDProportional 0.59
#define PIDIntegral 0
#define PIDDerivative 0.28
#define PIDIZone 0

#define driveTurningGain 0.25
#define driveProportional 0.9
#define driveIntegral 0
#define driveDerivitive 0.13

#define motorInitMaxCurrent 100 // The initial max current setting
#define motorInitRatedCurrent 60 // The inital rated current settings
#define motorInitLimitCycles 2000 // The inital number of allowed ms at peak current
#define lInvert false // Inversion setings for sides (invert this if opposite side)
#define rInvert true 

#define xDeadband 0.025
#define yDeadband 0.025
#define centerToWheel 1.08333 //Center of the robot to outer side of the wheel?
#define PI 3.141592654
#define wheelDiameter 4 //inches
#define maxOffsetAngle 1

// #define maxAcc = 7.0
// #define maxVelocity = 21.0

class DriveBaseModule{ //needed for gyroPIDDrive implementation

  AHRS *ahrs; //needs to be intialized in constructor

  ElevatorModule* elev = new ElevatorModule(10); //Elevator

  double maxAcc =  20.0;
  double maxVelocity = 30.0;
  double currentVelocity = 0;
  double robTheta = 0;
  double gyroOffsetVal = 0;
  double delta =10;
  double tuningPrevTime = 0;
  
  frc::Joystick* driverStick = new frc::Joystick(driverStickPort);
  //frc::Joystick* operatorStick = new frc::Joystick(operatorStickPort);
  rev::CANSparkMax* lMotor = new rev::CANSparkMax(lMotorLeaderID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* lMotorFollower = new rev::CANSparkMax(lMotorFollowerID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* rMotor = new rev::CANSparkMax(rMotorLeaderID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* rMotorFollower = new rev::CANSparkMax(rMotorFollowerID, rev::CANSparkMax::MotorType::kBrushless);
  //if you don't include getEncoder here, it doesn't build?
  rev::SparkMaxRelativeEncoder lEncoder = lMotor->GetEncoder();
  rev::SparkMaxRelativeEncoder rEncoder = rMotor->GetEncoder();
  rev::SparkMaxPIDController lPID = lMotor->GetPIDController();
  rev::SparkMaxPIDController rPID = rMotor->GetPIDController();

  bool initDriveMotor(rev::CANSparkMax* motor, rev::CANSparkMax* follower, bool invert); //loads initial values into motors such as current limit and phase direction
  bool setPowerBudget(rev::CANSparkMax* motor, float iPeak, float iRated, int limitCycles); //changes the current limits on the motors 
  bool setDriveCurrLimit(float iPeak, float iRated, int limitCycles);

  public: 
  std::thread driveThread;
  double stopAuto = false;
  DriveBaseModule() {
    ahrs = new AHRS(frc::SerialPort::kMXP);
    driveThread = std::thread(&DriveBaseModule::run, this); //initializing thread so can detach in robot init
  }
  void LimitRate(double& s, double& t);
  void arcadeDrive(double vel, double dir); //takes two values from the joystick and converts them into motor output %
  bool PIDDrive(float totalFeet, bool keepVelocity);
  bool PIDTurn(float angle, float radius, bool keepVelocity);
  void autonomousSequence();
  bool initPath();   
  void run();
  void runInit();
  void gyroDriving();
  void PIDTuning();
  void driveBaseTuning();
  double skim(double v); 
  double getGyroAngleAuto() { //will be positive
    double angle = ahrs->GetAngle();
    if(angle * gyroOffsetVal < 0) { //if signs are different
     return fabs(fabs(ahrs->GetAngle()) + fabs(gyroOffsetVal)); //handles the case if it switches from positive to negative of the gyro
    }
    return fabs(fabs(ahrs->GetAngle()) - fabs(gyroOffsetVal)); //should alwyas return positive because PIDTurn method requires (changes signs in setpoint)
   }

  //old system doesn't work, need to fix for radius
  struct pathPoint {
    float x;
    float y;
  };
  std::vector<pathPoint> straightLinePoints;
  pathPoint robPos;
   struct radiusTurnPoint {
    float angle;
    float radius;
  };
  std::vector<radiusTurnPoint> radiusTurnPoints;
  std::vector<bool> pathOrder; 

  float d = 0;
  float theta = 0;
  //bool PIDGyroTurn(float angle, float radius, float maxAcc, float maxVelocity);
  float prevTime; //all for limit rate
  float prev_value_speed;
  float prev_value_turn;

  char state = 't';

  //Target Relative Positioning

  std::vector<double> getCoords();

  void setTarget(double x, double y);
  
  void updatePos(double left, double right, double angle);

  double getAngleToTarget();
  
  double getDistanceToTarget();
  
  double range360(double inp);

  //TRP Variables

  double target_x = 0;
  double target_y = 10;
  double start_x = 0;
  double start_y = 0;
  std::vector<double> position{0, 0, 0};

  double current_x = start_x;
  double current_y = start_y;
  double current_theta;
  double last_Lencoder = 0;
  double last_Rencoder= 0;

  // End of TRP

 frc2::PIDController* rightStickPID = new frc2::PIDController(driveProportional, driveIntegral, driveDerivitive);

  private:
	    double m_out;
};

