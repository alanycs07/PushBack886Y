#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.h"

MotorGroup left_mg({-7, -8, -9});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
MotorGroup right_mg({1, 2, 11});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
Motor leftFrontMotor(-7);
MotorGroup Front_intake_mg({-12});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
Motor Redirect_mg(-20);
MotorGroup Rear_Intake_mg({-21});
Imu imu(6);
Optical opticalSensor(10);
adi::Pneumatics Scraper('H', false, false);
adi::Pneumatics Descore('F', false, false);
adi::Pneumatics Aligner('G', false, false);
bool ScraperIsExtend;
bool alinerState;
Controller master(pros::E_CONTROLLER_MASTER);
pros::Rotation vertical_sensor(-18);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_sensor, lemlib::Omniwheel::NEW_2,0.5); //distance tbd

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_mg, // left motor group
                              &right_mg, // right motor group
                              9.85, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller (forward backward)
lemlib::ControllerSettings lateral_controller(3.75, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller (turning)
lemlib::ControllerSettings angular_controller(3.25, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              33, // derivative gain (kD)
                                              2, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}
bool colorSorton = false; 
bool removeRed;
bool BallDectected;

void colorsort(){
    opticalSensor.set_led_pwm(100);
	pros::lcd::print(4, "Color: %d", colorSorton);
    while (colorSorton == true){
		if(removeRed){ 
			if (opticalSensor.get_hue() >= 0 && opticalSensor.get_hue() <= 20) {
				Redirect_mg.move(-127);
				BallDectected = true;
				delay(550);
				Redirect_mg.move(0);

			}else{
				BallDectected = false;
			}
		}else{ 
			if (opticalSensor.get_hue() >= 0 && opticalSensor.get_hue() >= 120) {
				Redirect_mg.move(-127);
				BallDectected = true;
				delay(550);
				Redirect_mg.move(0);

			}else{
				BallDectected = false;
			}
		}
		delay(20);

    }
}
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	chassis.calibrate(); // calibrate sensors
	//pros::Task sortingTask(colorsort); 

	pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
			//pros::lcd::print(3, "Color: %d", opticalSensor.get_hue());
			//pros::lcd::print(4, "Vertical Sensor: %i", vertical_sensor.get_position());
            // delay to save resources
            pros::delay(100);
        }
    });
}


/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	left_mg.set_brake_mode(pros::MotorBrake::hold);
	right_mg.set_brake_mode(pros::MotorBrake::hold);
	//redirect = the thing used in color sort
	//Rear intake is the back roller
		//rear intake negative is score on middle goal
	//skills
	chassis.setPose(0, 0, 0);
	Front_intake_mg.move(127);
	Redirect_mg.move(127);
	chassis.moveToPoint(-8.5, 29.5, 2000, {.maxSpeed=60}); //move to the first 4 balls
	chassis.turnToHeading(-140, 500);	
	chassis.moveToPoint(-32.4, 0, 1000);
	chassis.turnToHeading(180, 400); //aim for loading zone
	chassis.waitUntilDone();
	Scraper.set_value(true);
	delay(200);
	chassis.moveToPoint(-32.4, -6, 1000, {.minSpeed=50}); //move to the loading 
	chassis.waitUntilDone();
	delay(2000);
	chassis.moveToPoint(-33.1, 24, 2000, {.forwards = false}); // score
	chassis.waitUntilDone();
	Rear_Intake_mg.move(127);
	Redirect_mg.move(127);
	delay(3200);
	Rear_Intake_mg.move(0);
	Redirect_mg.move(0);
	chassis.moveToPose(-10.5, 6, -270, 1800, {.forwards=true,.minSpeed=50});
	Scraper.set_value(false);
	chassis.turnToHeading(0, 500);
	chassis.moveToPoint(-10.5, 58, 2000, {.minSpeed=50}); // 2nd 4 ball stack
	chassis.waitUntilDone();
	delay(600);
	chassis.moveToPoint(-34, 100, 2000, {.minSpeed=50});
	chassis.waitUntilDone();
	delay(100);
	chassis.turnToHeading(0, 1000);
	chassis.waitUntilDone();
	Scraper.set_value(true);
	delay(200);
	chassis.moveToPoint(-36.4, 114, 1200, {.minSpeed=50}); //loading zone
	chassis.waitUntilDone();
	delay(2000);
	chassis.moveToPoint(-35.5, 86.5, 2000, {.forwards = false}); //score
	chassis.waitUntilDone();
	Rear_Intake_mg.move(127);
	Redirect_mg.move(127);
	delay(3200);
	Rear_Intake_mg.move(0);
	Redirect_mg.move(0);
	chassis.moveToPoint(-34, 107, 1000);
	chassis.turnToHeading(-255, 600);
	chassis.moveToPoint(41.5, 81.5, 2000, {.minSpeed=40}); // 3rd 4 ball
	Scraper.set_value(false);
	chassis.turnToHeading(30, 600);
	chassis.moveToPoint(65, 105, 1200, {.minSpeed=20});
	chassis.turnToHeading(0, 400);
	chassis.waitUntilDone();
	Scraper.set_value(true);
	delay(200);
	chassis.moveToPoint(65.6, 119.5, 1000, {.minSpeed=50}); //loading zone
	chassis.waitUntilDone();
	delay(1500);
	chassis.moveToPoint(66.5, 89, 1500, {.forwards = false}); 
	chassis.waitUntilDone();
	Rear_Intake_mg.move(127);
	Redirect_mg.move(127);
	delay(3200);
	Rear_Intake_mg.move(0);
	Redirect_mg.move(0);
	chassis.moveToPose(40, 107, -90, 1800, {.forwards=true,.minSpeed=50});
	chassis.turnToHeading(180, 600);
	chassis.moveToPose(40, 40, 180, 1800, {.forwards=true,.minSpeed=50});
	

	


	// solo awp
	// chassis.setPose(-46, 0, 90);
	// Front_intake_mg.move(127);
	// Redirect_mg.move(127);
	// chassis.moveToPose(-22,21,45,500,{.forwards=true,.maxSpeed=90,.minSpeed=80});
	// chassis.moveToPose(-21,21,50,1250,{.forwards=true,.maxSpeed=90,.minSpeed=40});
	// chassis.waitUntilDone();
	// pros::delay(1000);

	// chassis.turnToHeading(315, 800);	
	// chassis.moveToPose(-4, 8, 315, 100, {.forwards=false,.minSpeed=60});
	// chassis.moveToPose(-5.5, 9.5, 315, 600, {.forwards=false,.minSpeed=30});
	// chassis.waitUntilDone();
	// Rear_Intake_mg.move(-127);
	// delay(1500);
	// chassis.moveToPose(-46, 48.5,270, 4000, {.forwards=true,.lead=0.3,.minSpeed=30,});
	// Aligner.extend();
	// Rear_Intake_mg.move(0);
	// chassis.waitUntilDone();
	// chassis.turnToHeading(270, 	350);
	// Scraper.set_value(true);
	// chassis.waitUntilDone();
	// chassis.cancelAllMotions();
	// Rear_Intake_mg.move(0);
	// Front_intake_mg.move(127);
	// chassis.arcade(45, 0);
	// delay(950);

	// chassis.moveToPoint(-20, 50, 1000, {.forwards = false,.maxSpeed=100});
	// chassis.waitUntilDone();
	// Rear_Intake_mg.move(127);
	// delay(2000);

	// Scraper.retract();
	// Aligner.retract();


	// // comment this out if you want to stay at high goal (red half awp)

	// chassis.moveToPose(-22, -24, 180, 5000, {.lead=0.2, .minSpeed=10});
	// chassis.waitUntil(68);
	// Front_intake_mg.move(0);
	// chassis.waitUntilDone();

	// chassis.turnToPoint(-12, -9, 750);
	// chassis.waitUntilDone();
	// delay(200);
	// Front_intake_mg.move(-127);
	// chassis.moveToPoint(0, 0, 500, {.maxSpeed=40});







	// // awp
	// Aligner.set_value(false); //aligner extend
	// removeRed = true; //false for red side, true for blue side
	// chassis.setPose(-50, 0, 90);
	// Front_intake_mg.move(127);
	// Redirect_mg.move(127);
	// chassis.moveToPoint(-29,21,1000,{.maxSpeed=69}); //-30, 19
	// chassis.moveToPoint(-24, 25.7,750, {.maxSpeed = 40}); //collect the two blocks
	// chassis.moveToPoint(-20, 24,200, {.maxSpeed = 45}); //collect the two blocks
	// chassis.turnToPoint(-50,49.5, 800,{.forwards = false, .minSpeed = 50});
	// // chassis.moveToPoint(-29,21,1000,{.maxSpeed=69}); //-30, 19
	// // chassis.moveToPoint(-24, 25.7,750, {.maxSpeed = 40}); //collect the two blocks
	// // chassis.moveToPoint(-20, 24,200, {.maxSpeed = 45}); //collect the two blocks
	// // chassis.turnToPoint(-50,49.5, 800,{.forwards = false, .minSpeed = 50});
	// chassis.moveToPoint(-50,51,800,{.forwards = false, .minSpeed = 50}); //half to hg
	// chassis.turnToHeading(270,700,{.maxSpeed = 80});
	// Redirect_mg.move(127);
	// chassis.moveToPoint(-28, 49.5, 1000, {.forwards = false, .maxSpeed = 50}); //go to hg
	// chassis.waitUntilDone();
	// Rear_Intake_mg.move(127); //score
	// Redirect_mg.move(127);
	// pros::delay(1700); //wait for scoring
	// Rear_Intake_mg.move(0);
	// Redirect_mg.move(0);
	// Aligner.set_value(true); //aligner retract
	// Scraper.set_value(true); //put scraper down
	// chassis.moveToPoint(-59, 50.5, 1000, {.minSpeed=50}); //drive to loader
	// chassis.moveToPoint(-59, 49.5, 1000, {.maxSpeed = 65,.minSpeed = 40}); //drive to loader
	// pros::delay(1400);
	// Redirect_mg.move(0);
	// Scraper.set_value(false);
	// chassis.moveToPoint(-17,17,800,{.forwards = false, .minSpeed = 50}); //drive towards center goal
	// chassis.moveToPoint(-13,12,1000,{.forwards= false,.maxSpeed = 40, .minSpeed = 20}); //go to middle goal
	// chassis.waitUntilDone();
	// Rear_Intake_mg.move(-100);
	// // Aligner.set_value(false);
	// Redirect_mg.move(127); 
	// pros::delay(1400);//score on midle goal
	

	
	// //split here for hgmg
	// Rear_Intake_mg.move(0);
	// chassis.moveToPoint(-21, 7, 750,{.minSpeed=100}); //location halfway to far blocks
	// chassis.moveToPoint(-23, -28, 1000,{.maxSpeed = 65}); //location of far blocks
	// Rear_Intake_mg.move(0);
	// chassis.moveToPose(-10,-14, 235, 1000, {.forwards = false, .minSpeed = 80}, true);
	/*chassis.turnToHeading(225, 700); //shoudl be 225
	chassis.waitUntilDone();
	// chassis.moveToPoint(-17, -16, 1000,{.forwards=false}); //location low goal*/
	// chassis.waitUntilDone();
	// Rear_Intake_mg.move(-90); //score on low goal

	// 7 ball left side
	// Aligner.set_value(false); //aligner extend
	// removeRed = false; //false for red side, true for blue side
	// chassis.setPose(-50, 0, 90);
	// Front_intake_mg.move(127);
	// Redirect_mg.move(127);
	// chassis.moveToPoint(-29,21,1000,{.maxSpeed=69}); //-30, 19
	// chassis.moveToPoint(-24, 25.5,750, {.maxSpeed = 40}); //collect the two blocks
	// chassis.moveToPoint(-20, 24,200, {.maxSpeed = 45}); //collect the two blocks
	// chassis.turnToPoint(-50,49.5, 800,{.forwards = false, .minSpeed = 50});
	// chassis.moveToPoint(-50,51,800,{.forwards = false, .minSpeed = 50}); //half to hg
	// chassis.turnToHeading(270,700,{.maxSpeed = 80});
	// Redirect_mg.move(127);
	// chassis.moveToPoint(-28, 49.5, 1000, {.forwards = false, .maxSpeed = 50}); //go to hg
	// chassis.waitUntilDone();
	// Rear_Intake_mg.move(127); //score
	// Redirect_mg.move(127);
	// pros::delay(1700); //wait for scoring
	// Rear_Intake_mg.move(0);
	// Redirect_mg.move(0);
	// Aligner.set_value(true); //aligner retract
	// Scraper.set_value(true); //put scraper down
	// chassis.moveToPoint(-59, 50.5, 1000, {.minSpeed=50}); //drive to loader
	// chassis.moveToPoint(-59, 49.5, 1000, {.maxSpeed = 65,.minSpeed = 40}); //drive to loader
	// // pros::delay(1400);
	// 	pros::delay(2000);

	// Redirect_mg.move(0);
	// Scraper.set_value(false);
	// Aligner.set_value(false);
	// chassis.moveToPoint(-26, 49.5, 1700, {.forwards = false, .maxSpeed = 50}); //go to hg
	// chassis.waitUntilDone();
	// Rear_Intake_mg.move(127); //score
	// Redirect_mg.move(127);
	// // pros::delay(1700);
	// pros::delay(300);
	// Rear_Intake_mg.move(0);
	// Redirect_mg.move(0);
	// chassis.moveToPoint(-45,51,800);
	// chassis.turnToHeading(58,800);
	// Descore.set_value(true);
	// chassis.moveToPoint(-17,63,2000,{.maxSpeed = 70});
	// chassis.moveToPoint(-11,63,2000,{.maxSpeed = 80,.minSpeed=50});



	//7 ball right side unfinished
	// Aligner.set_value(false); //aligner extend
	// removeRed = false; //false for red side, true for blue side
	// chassis.setPose(-50, 0, 90);
	// Front_intake_mg.move(127);
	// Redirect_mg.move(127);
	// chassis.moveToPoint(-29,-21,1000,{.maxSpeed=100}); //-30, 19
	// chassis.moveToPoint(-24, -28.5,750, {.maxSpeed = 60}); //collect the two blocks
	// chassis.moveToPoint(-20, -24,200, {.maxSpeed = 80}); //collect the two blocks
	// chassis.turnToPoint(-50,-49.5, 800,{.forwards = false, .minSpeed = 75});
	// chassis.moveToPoint(-50,-51,800,{.forwards = false, .minSpeed = 80}); //half to hg
	// chassis.turnToHeading(270,700,{.maxSpeed = 80});
	// Redirect_mg.move(127);
	// Scraper.set_value(true); //put scraper down
	// chassis.moveToPoint(-65, -50.5, 500, {.minSpeed=50}); //drive to loader
	// chassis.moveToPoint(-56, -53.5, 1000, {.maxSpeed = 65,.minSpeed = 20}); //drive to loader
	// // pros::delay(1400);
	// pros::delay(575);
	// Redirect_mg.move(0);
	// Scraper.set_value(false);
	// Aligner.set_value(false);
	// chassis.moveToPoint(-28, -50, 1300, {.forwards = false, .maxSpeed = 80}); //go to hg
	// chassis.waitUntilDone();
	// Rear_Intake_mg.move(127); //score
	// Redirect_mg.move(127);
	// pros::delay(2000);
	// // pros::delay(300);
	// Rear_Intake_mg.move(0);
	// Redirect_mg.move(0);
	// chassis.moveToPoint(-45,-51,800);
	// chassis.turnToHeading(32,800);
	// Descore.set_value(true);
	// chassis.moveToPoint(-32.5,-39.5,1000,{.maxSpeed = 90});
	// chassis.turnToHeading(90, 1000);
	// chassis.moveToPoint(-10, chassis.getPose().y, 500, {.minSpeed=127});
	// chassis.waitUntil(4.75);
	// chassis.cancelAllMotions();
	// chassis.arcade(0, 0);

	

}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {


	bool alignerMech = false;
	bool scrapperMech = true;
		Aligner.set_value(false);
		Scraper.set_value(false);
		/*pros::Task([&]{
			while(true){
				colorsort();
				delay(10);
			}
		});*/
	while (true) {
		// pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		//                  (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		//                  (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs
		// Arcade control scheme

		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_LEFT_X);  // Gets the turn left/right from right joystick
		left_mg.move(dir + 0.70*turn);                      // Sets left motor voltage
		right_mg.move(dir - 0.70*turn);                     // Sets right motor voltage
		removeRed = true;
		// if (master.get_digital(DIGITAL_L1)) {
		// 	Front_intake_mg.move(100);
		// 	opticalSensor.set_led_pwm(100);
		// 	colorSorton = true; 

		// 	if (BallDectected == false) {
		// 		Redirect_mg.move(127);

		// 	}
        	

    	// //}
		// // else if (master.get_digital(DIGITAL_R2)) {
		// // 	Front_intake_mg.move(-100);
		// // }
		// }else {
		// 	Front_intake_mg.move(0);
		// 	if (BallDectected == false) {
		// 		Redirect_mg.move(0);

		// 	}
		// 	//colorSorton = false; 
		// }
	

		if (master.get_digital(DIGITAL_R1)) {
			Rear_Intake_mg.move(127);
			//Redirect_mg.move(127);
			Redirect_mg.move(127);
		}
		else if (master.get_digital(DIGITAL_L2)) {
			Rear_Intake_mg.move(-127);
			//Redirect_mg.move(127);
			Redirect_mg.move(127);
		}
		else {
			Rear_Intake_mg.move(0);
			Redirect_mg.move(0);
			
		}

		if (master.get_digital(DIGITAL_L1)) {
			Front_intake_mg.move(127);
			Redirect_mg.move(127);
		}
		else if (master.get_digital(DIGITAL_R2)) {
			Front_intake_mg.move(-127);
			
		}
		else {
			Front_intake_mg.move(0);
			
		}

		if (master.get_digital_new_press(DIGITAL_Y)){
			scrapperMech = !scrapperMech;
			alignerMech = !alignerMech;

			if(alignerMech){
				Scraper.set_value(scrapperMech);
				Aligner.set_value(!alignerMech);
			}
			else if(!alignerMech && !scrapperMech){
				Scraper.set_value(scrapperMech);
			}
			else{
				Scraper.set_value(scrapperMech);
				Aligner.set_value(alignerMech);
			}


		}

		if (master.get_digital_new_press(DIGITAL_A)) {

			Descore.toggle();
		}

		if (master.get_digital_new_press(DIGITAL_RIGHT)){
			alignerMech = !alignerMech;
			Aligner.set_value(alignerMech);
		}   
		
		// if(master.get_digital_new_press(DIGITAL_LEFT)){
		// 	Scraper.toggle();
		// }

		pros::delay(25);                               // Run for 20 ms then update
	}
}