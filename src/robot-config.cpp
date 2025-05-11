#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen.
brain  Brain;

//The motor constructor takes motors as (port, ratio, reversed), so for example
//motor LeftFront = motor(PORT1, ratio6_1, false);

//Add your devices below, and don't forget to do the same in robot-config.h:

motor LeftFront = motor(PORT1, ratio6_1, false);
motor LeftBack = motor(PORT2, ratio6_1, false);
motor RightFront = motor(PORT3, ratio6_1, false);
motor RightBack = motor(PORT4, ratio6_1, false);
motor leftMiddle = motor(PORT5, ratio6_1, true);
motor rightMiddle = motor(PORT6, ratio6_1, true);
motor intake = motor(PORT7, ratio18_1, false);
motor scorer = motor(PORT8, ratio18_1, false);

void vexcodeInit( void ) {
  // nothing to initialize
}