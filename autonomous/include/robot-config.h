using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor Motor3;
extern motor Motor4;
extern inertial Inertial20;
extern motor intakeMotor1;
extern motor intakeMotor2;
extern motor roller1;
extern motor roller2;
extern motor Motor1;
extern motor Motor2;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );