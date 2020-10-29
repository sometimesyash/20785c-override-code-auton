#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor Motor3 = motor(PORT3, ratio18_1, true);
motor Motor4 = motor(PORT4, ratio18_1, true);
inertial Inertial20 = inertial(PORT20);
motor intakeMotor1 = motor(PORT5, ratio6_1, false);
motor intakeMotor2 = motor(PORT6, ratio6_1, false);
motor roller1 = motor(PORT7, ratio18_1, false);
motor roller2 = motor(PORT8, ratio18_1, true);
motor Motor1 = motor(PORT1, ratio18_1, false);
motor Motor2 = motor(PORT2, ratio18_1, false);

// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}