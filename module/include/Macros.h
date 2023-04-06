// ModuleID Definitions for use in constructorArgs

#define ErrorModuleID 0
#define DriveBaseModuleID 1
#define AutonomousModuleID 2
#define IntakeModuleID 3
#define ShooterModuleID 4

// Error loglevel
#define INFO 0
#define LOW 1
#define MEDIUM 2
#define HIGH 3
#define FATAL 4

// Frequency of running periodicRoutine()
#define ErrorModuleRunInterval 80 
#define DriveBaseModuleRunInterval 20 
#define ControllerModuleRunInterval 35
#define AutonomousModuleRunInterval 30
#define IntakeModuleRunInterval 30
#define ShooterModuleRunInterval 25

//motorIDs
#define elevatorID 13
#define scaraArmInner 12
#define scaraArmOutter 14
#define lMotorLeaderID 1
#define lMotorFollowerID 2
#define rMotorLeaderID 4
#define rMotorFollowerID 3
#define grabberID 23
#define rotaryID 9
#define leftStarID 15
#define rightStarID 17
#define rollerID 16 //THIS WAS CHANGED FROM 16
// #define testLeft 14
// #define testRight 1