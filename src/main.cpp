#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/misc.h"
#include <chrono>

MotorGroup left_mg({-11, -17, -18});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
MotorGroup right_mg({14, 15, 16});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
Motor leftFrontMotor(-7);
MotorGroup Front_intake_mg({-13});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
Motor Redirect_mg(-2);
MotorGroup Top_Intake({20});
Imu imu(12);
Optical opticalSensor(10);
adi::Pneumatics Scraper('A', false, false);
adi::Pneumatics Descore('B', true, false);
adi::Pneumatics Trapdoor('C', false, false);
bool ScraperIsExtend;
bool alinerState;
Controller master(pros::E_CONTROLLER_MASTER);
pros::Rotation vertical_sensor(21);
pros::Rotation horizontal_sensor(9);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_sensor, lemlib::Omniwheel::NEW_275, 0); //distance tbd
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_sensor, lemlib::Omniwheel::NEW_2, 2.5); //distance tbd

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
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller (forward backward)
lemlib::ControllerSettings lateral_controller(6.2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              20, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in inches
                                              10, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller (turning)
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              1, // anti windup
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
			}else if(opticalSensor.get_hue() >= 200 && opticalSensor.get_hue() < 250 && BallDectected){
				BallDectected = false;
			}
		}else{ 
			if (opticalSensor.get_hue() >= 0 && opticalSensor.get_hue() >= 120) {
				Redirect_mg.move(-127);
				BallDectected = true;

			}else if (opticalSensor.get_hue() >= 0 && opticalSensor.get_hue() <= 20) {
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
	opticalSensor.set_integration_time(20);
	Descore.set_value(true);

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
	// pros::Task([=]{
	// 	Front_intake_mg.move(-127);
	// 	delay(200);
	// 	Front_intake_mg.move(127);
	// });


	// //right side
	// chassis.setPose(46.5, 16.5, 270);
	// Front_intake_mg.move(127);
	// chassis.moveToPoint(12, 25, 1100, {.forwards = true, .maxSpeed = 75});
	// chassis.moveToPoint(17, 23, 700, {.forwards = false, .minSpeed = 30});
	// chassis.turnToHeading(-135, 700);
	// Front_intake_mg.move(0);
	// chassis.moveToPoint(9, 20.5, 800);
	// chassis.waitUntilDone();
	// Front_intake_mg.move(-127);
	// delay(1000);
	// chassis.moveToPoint(7, 18, 500, {.minSpeed=50});
	// chassis.waitUntilDone();
	// Front_intake_mg.move(0);
	// chassis.moveToPoint(32, 51, 1500, {.forwards = false, .maxSpeed = 90});
	// chassis.turnToHeading(90, 900);
	// chassis.waitUntilDone();
	// Scraper.extend();
	// chassis.moveToPoint(55, 48, 800, {.forwards = true, .maxSpeed = 85});
	// Front_intake_mg.move(127);
	// chassis.waitUntilDone();
	// delay(700);
	// Front_intake_mg.move(0);
	// chassis.moveToPoint(10, 49, 1000, {.forwards = false, .maxSpeed = 90});
	// chassis.waitUntilDone();
	// Front_intake_mg.move(127);
	// Top_Intake.move(127);
	// delay(2100);
	// chassis.moveToPoint(27, 49.5, 800, {.forwards = true, .minSpeed = 20});
	// chassis.moveToPoint(10, 49.5, 1500, {.forwards = false, .minSpeed = 127});


	//left side

	// chassis.setPose(46.5, -16.5, 270);
	// Front_intake_mg.move(127);
	// chassis.moveToPoint(12, -25, 1100, {.forwards = true, .maxSpeed = 65});
	// chassis.waitUntilDone();
	// delay(200);
	// chassis.moveToPoint(17, -23, 700, {.forwards = false, .minSpeed = 30});
	// chassis.waitUntilDone();
	// delay(100);
	// chassis.turnToHeading(128, 700);
	// Front_intake_mg.move(0);
	// chassis.moveToPoint(-0.8, -8.3,800, {.forwards=false});
	// chassis.waitUntilDone();
	// Top_Intake.move(80);
	// Front_intake_mg.move(95);
	// Trapdoor.extend();
	// delay(2000);
	// Front_intake_mg.move(0);
	// Top_Intake.move(0);
	// chassis.moveToPoint(32, -43.2, 1200, {.forwards = true, .maxSpeed = 90});
	// chassis.turnToHeading(90, 600);
	// chassis.waitUntilDone();
	// Scraper.extend();
	// Trapdoor.retract();
	// delay(300);
	// chassis.moveToPoint(58, -46.5, 800, {.forwards = true, .minSpeed = 60});
	// Front_intake_mg.move(127);
	// chassis.waitUntilDone();
	// delay(700);
	// Front_intake_mg.move(0);
	// chassis.moveToPoint(10, -49, 1000, {.forwards = false, .maxSpeed = 90});
	// chassis.waitUntilDone();
	// Front_intake_mg.move(127);
	// Top_Intake.move(127);
	// delay(2200);
	// chassis.moveToPoint(27, -48.5, 800, {.forwards = true, .minSpeed = 20});
	// chassis.moveToPoint(10, -48.5, 1500, {.forwards = false, .minSpeed = 127});
	// Scraper.retract();


	//elims left side

	// chassis.setPose(46.5, -16.5, 270);
	// Front_intake_mg.move(127);
	// chassis.moveToPoint(12, -26, 1100, {.forwards = true, .maxSpeed = 60});
	// chassis.moveToPose(1.5, -45, 180, 2400, {.forwards = true, .lead = 0.2, .maxSpeed = 35});
	// chassis.waitUntilDone();
	// delay(200);
	// chassis.moveToPoint(17, -35, 1200, {.forwards = false, .minSpeed = 30});
	// chassis.waitUntilDone();
	// delay(100);
	// chassis.turnToHeading(110, 900);
	// Scraper.retract();
	// chassis.waitUntilDone();
	// chassis.moveToPose(32, -48, 125, 1000, {.forwards = true, .lead = 0.15, .maxSpeed = 90, .minSpeed = 40, .earlyExitRange = 2});
	// chassis.turnToHeading(90, 600);
	// chassis.waitUntilDone();
	// Scraper.extend();
	// Trapdoor.retract();
	// delay(500);
	// chassis.moveToPoint(58, -53, 800, {.forwards = true, .maxSpeed = 60});
	// Front_intake_mg.move(127);
	// chassis.waitUntilDone();
	// delay(500);
	// Front_intake_mg.move(0);
	// chassis.moveToPoint(10, -51, 1000, {.forwards = false, .maxSpeed = 60});
	// chassis.waitUntilDone();
	// Front_intake_mg.move(127);
	// Top_Intake.move(127);
	// delay(2200);
	// chassis.moveToPoint(27, -52 , 800, {.forwards = true, .minSpeed = 20});
	// chassis.moveToPoint(10, -52, 1500, {.forwards = false, .minSpeed = 127});
	// Scraper.retract();

	//left 2 ball middle goal, 7 ball tall (elims)

	// chassis.setPose(46.5, -16.5, 270);
	// Front_intake_mg.move(127);
	// chassis.moveToPoint(12, -26, 1100, {.forwards = true, .maxSpeed = 60});
	// chassis.moveToPose(0.3, -45, 180, 2400, {.forwards = true, .lead = 0.2, .maxSpeed = 35});
	// chassis.waitUntilDone();
	// delay(200);
	// chassis.moveToPoint(17.3, -23, 700, {.forwards = false, .minSpeed = 30});
	// chassis.waitUntilDone();
	// delay(100);
	// chassis.turnToHeading(132, 800);
	// Front_intake_mg.move(0);
	// chassis.moveToPoint(1, -10,800, {.forwards=false});
	// chassis.waitUntilDone();
	// Top_Intake.move(80);
	// Front_intake_mg.move(95);
	// Trapdoor.extend();
	// delay(600);
	// Front_intake_mg.move(0);
	// Top_Intake.move(0);
	// chassis.moveToPose(32, -44, 125, 1000, {.forwards = true, .lead = 0.15, .maxSpeed = 90, .minSpeed = 40});
	// chassis.waitUntilDone();
	// delay(100);
	// chassis.turnToHeading(90, 600);
	// chassis.waitUntilDone();
	// Scraper.extend();
	// Trapdoor.retract();
	// delay(470);
	// chassis.moveToPoint(58, -49.4, 800, {.forwards = true, .maxSpeed = 60});
	// Front_intake_mg.move(127);
	// chassis.waitUntilDone();
	// delay(500);
	// Front_intake_mg.move(0);
	// chassis.moveToPoint(10, -50.4, 1000, {.forwards = false, .maxSpeed = 60});
	// chassis.waitUntilDone();
	// Front_intake_mg.move(127);
	// Top_Intake.move(127);
	// delay(2200);
	// chassis.moveToPoint(27, -51 , 800, {.forwards = true, .minSpeed = 20});
	// chassis.moveToPoint(10, -50.4, 1500, {.forwards = false, .minSpeed = 127});
	// Scraper.retract();

	//right side 2 ball middle goal, 7 ball tall

	// chassis.setPose(46.5, 16.5, 270);
	// Front_intake_mg.move(127);
	// chassis.moveToPoint(9.8, 26, 1400, {.forwards = true, .maxSpeed = 60});
	// chassis.moveToPose(0.5, 45, 0, 2300, {.forwards = true, .lead = 0.2, .maxSpeed = 40});
	// chassis.waitUntilDone();
	// delay(200);
	// chassis.moveToPose(19, 21, -45, 1600, {.forwards = false, .lead = 0.7, .minSpeed = 50});
	// chassis.waitUntilDone();
	// delay(100);
	// chassis.turnToHeading(218, 700);
	// Front_intake_mg.move(0);
	// chassis.moveToPoint(8.5, 17.5, 900, {.maxSpeed = 60});
	// chassis.waitUntilDone();
	// Front_intake_mg.move(-100);
	// delay(900);
	// Front_intake_mg.move(10);


	// chassis.moveToPose(34, 51.5, 125, 1600, {.forwards = false, .lead = 0.15, .maxSpeed = 100, .minSpeed = 50});
	// chassis.turnToHeading(90, 600);
	// chassis.waitUntilDone();
	// Scraper.extend();
	// Trapdoor.retract();
	// delay(450);
	// chassis.moveToPoint(58, 50.4, 800, {.forwards = true, .maxSpeed = 60});
	// Front_intake_mg.move(127);
	// chassis.waitUntilDone();
	// delay(500);
	// Front_intake_mg.move(0);
	// chassis.moveToPoint(10, 50.4, 1000, {.forwards = false, .maxSpeed = 60});
	// chassis.waitUntilDone();
	// Front_intake_mg.move(127);
	// Top_Intake.move(127);
	// delay(2200);
	// chassis.moveToPoint(27, 50.9 , 800, {.forwards = true, .minSpeed = 20});
	// chassis.moveToPoint(10, 50.9, 1500, {.forwards = false, .minSpeed = 127});
	// Scraper.retract();

	//right side 9 ball

	// chassis.setPose(46.5, 16.5, 270);
	// Front_intake_mg.move(127);
	// chassis.moveToPoint(9.8, 26, 1400, {.forwards = true, .maxSpeed = 60});
	// chassis.moveToPose(0.5, 45, 0, 2300, {.forwards = true, .lead = 0.2, .maxSpeed = 40});
	// chassis.waitUntilDone();
	// delay(200);
	// chassis.moveToPose(19, 21, -45, 1600, {.forwards = false, .lead = 0.7, .minSpeed = 50});
	// chassis.waitUntilDone();
	// delay(100);
	// chassis.turnToHeading(50, 900);


	// chassis.moveToPose(34, 44, 65, 1600, {.forwards = true, .lead = 0.15, .maxSpeed = 100, .minSpeed = 50, .earlyExitRange = 1});
	// chassis.turnToHeading(90, 600);
	// chassis.waitUntilDone();
	// Scraper.extend();
	// Trapdoor.retract();
	// delay(450);
	// chassis.moveToPoint(58, 50, 800, {.forwards = true, .maxSpeed = 60});
	// Front_intake_mg.move(127);
	// chassis.waitUntilDone();
	// delay(500);
	// Front_intake_mg.move(0);
	// chassis.moveToPoint(10, 50, 1000, {.forwards = false, .maxSpeed = 60});
	// chassis.waitUntilDone();
	// Front_intake_mg.move(127);
	// Top_Intake.move(127);
	// delay(2200);
	// chassis.moveToPoint(27, 50.4 , 800, {.forwards = true, .minSpeed = 20});
	// chassis.moveToPoint(10, 50.4, 1500, {.forwards = false, .minSpeed = 127});
	// Scraper.retract();


	//solo awp

	// chassis.setPose(46.5, -16.5, 180);
	// chassis.moveToPoint(45, -34.8, 800, {.minSpeed = 40, .earlyExitRange = 2});
	// Descore.retract();
	// chassis.swingToHeading(90, DriveSide::LEFT, 700);
	// chassis.waitUntilDone();
	// Scraper.extend();
	// Trapdoor.retract();
	// delay(330);
	// chassis.moveToPoint(54, -53, 500);
	// Front_intake_mg.move(127);
	// chassis.waitUntilDone();
	// delay(290);
	// chassis.moveToPoint(17, -54, 1000, {.forwards = false, .maxSpeed = 90});
	// chassis.waitUntilDone();
	// Top_Intake.move(127);
	// delay(1010);
	// Top_Intake.move(0);
	// Scraper.retract();
	// chassis.swingToHeading(-45, DriveSide::LEFT, 900, {.maxSpeed = 80});
	// chassis.moveToPoint(20, -30 , 800, {.forwards = true});
	// chassis.turnToHeading(138, 800);
	// chassis.waitUntilDone();
	// delay(30);
	// chassis.moveToPoint(1.4, -14 , 700, {.forwards = false});
	// Top_Intake.move(-80);
	// Front_intake_mg.move(0);
	// chassis.waitUntilDone();
	// Top_Intake.move(110);
	// Front_intake_mg.move(110);
	// Trapdoor.extend();
	// delay(600);
	// Top_Intake.move(0);
	// // chassis.moveToPoint(13, -24 , 800, {.forwards = true, .minSpeed = 20});
	// Trapdoor.retract();
	// // chassis.turnToHeading(0, 500, {.earlyExitRange = 3});
	// chassis.moveToPoint(15.5, 12 , 1500, {.forwards = true, .maxSpeed = 80, .minSpeed = 20, .earlyExitRange = 3});
	// chassis.turnToPoint(40, 38, 300, {.earlyExitRange = 3});
	// chassis.moveToPoint(40, 38, 1000, {.forwards = true, .maxSpeed = 70, .minSpeed = 20});
	// chassis.turnToHeading(90, 400);
	// chassis.waitUntilDone();
	// Scraper.extend();
	// delay(150);
	// chassis.moveToPoint(57, 39, 800, {.maxSpeed = 100});
	// chassis.waitUntilDone();
	// delay(350);
	// chassis.moveToPoint(9, 40, 800, {.forwards = false, .maxSpeed = 80});
	// chassis.waitUntilDone();
	// Front_intake_mg.move(127);
	// Top_Intake.move(127);

	


	// NEW SKILLS ____________________________
	
	chassis.setPose(-48, 16, 90);
	Descore.retract();
	Front_intake_mg.move(127);
	chassis.moveToPoint(-19, 23, 2000, {.maxSpeed=56}); //move to the first 4 balls
	chassis.turnToHeading(320, 800);	
	chassis.moveToPoint(-30, 43, 1000);
	chassis.turnToPoint(-52, 48, 900); //aim for loading zone
	chassis.waitUntilDone();
	Scraper.set_value(true);
	delay(100);
	chassis.moveToPoint(-52, 48.7, 1000, {.minSpeed = 20}); //move to the loading 
	chassis.waitUntilDone();
	delay(1500);
	chassis.moveToPoint(-13, 50.3, 1500, {.forwards = false, .maxSpeed = 70}); // score
	chassis.waitUntilDone();
	Front_intake_mg.move(127);
	Top_Intake.move(127);
	delay(3100);
	Front_intake_mg.move(0);
	Top_Intake.move(0);
	Scraper.set_value(false);
	chassis.moveToPose(-35, 26, 180, 1000, {.forwards=true, .lead = 0.2, .maxSpeed = 70});
	chassis.waitUntilDone();
	delay(200);
	chassis.turnToHeading(90, 800);
	Front_intake_mg.move(127);
	chassis.waitUntilDone();
	delay(200);
	chassis.moveToPoint(28, 25, 1800, {.forwards=true, .maxSpeed = 100}); //move to 2nd stack
	chassis.waitUntilDone();
	delay(200);
	chassis.moveToPose(48, 48.2, 45, 1400, {.forwards=true, .lead = 0.2, .maxSpeed = 50});
	chassis.turnToHeading(90, 800);
	chassis.waitUntilDone();
	Scraper.set_value(true);
	delay(600);
	chassis.moveToPoint(66, 48, 800, {.forwards=true, .maxSpeed = 90}); //loading 2
	chassis.waitUntilDone();
	delay(1500);
	chassis.moveToPoint(23, 49.2, 1800, {.forwards = false, .maxSpeed = 70}); // score
	chassis.waitUntilDone();
	Front_intake_mg.move(127);
	Top_Intake.move(127);
	delay(3100);
	Front_intake_mg.move(0);
	Top_Intake.move(0);
	Scraper.set_value(false);
	chassis.moveToPoint(35, 52, 800, {.minSpeed = 70}); 
	chassis.turnToPoint(33, -19, 800);
	chassis.moveToPoint(33, -19, 2000, {.maxSpeed = 90});
	Front_intake_mg.move(127);
	chassis.turnToPoint(52, -42.7, 800);
	chassis.moveToPoint(52, -42.7, 800, {.forwards=true, .maxSpeed = 70});
	chassis.turnToHeading(90, 800);
	chassis.waitUntilDone();
	Scraper.set_value(true);
	delay(500);
	chassis.moveToPoint(64, -44, 800, {.forwards=true});
	chassis.waitUntilDone();
	delay(2000);
	chassis.moveToPoint(23, -46, 1000, {.forwards = false, .maxSpeed = 70}); // score
	chassis.waitUntilDone();
	Front_intake_mg.move(127);
	Top_Intake.move(127);
	delay(3100);
	Front_intake_mg.move(0);
	Top_Intake.move(0);
	Scraper.set_value(false);
	chassis.moveToPose(40, -23.5, 0, 1500, {.forwards=true, .lead = 0.2, .maxSpeed = 60});
	chassis.waitUntilDone();
	delay(500);
	chassis.turnToHeading(-90, 800);
	Front_intake_mg.move(127);
	chassis.moveToPoint(-2, -23.5, 1500, {.forwards=true, .maxSpeed = 90});
	chassis.moveToPose(-33, -44, 230, 1500, {.forwards=true, .lead = 0.3, .maxSpeed = 70});
	chassis.turnToHeading(-90, 800);
	chassis.waitUntilDone();
	Scraper.set_value(true);
	delay(600);
	chassis.moveToPoint(-52, -46.3, 1500, {.forwards=true, .maxSpeed = 90});
	Front_intake_mg.move(127);
	chassis.waitUntilDone();
	delay(1500);
	Front_intake_mg.move(0);
	chassis.moveToPoint(-5, -47, 1500, {.forwards=false, .maxSpeed = 70});
	chassis.waitUntilDone();
	Front_intake_mg.move(127);
	Top_Intake.move(127);
	delay(3100);
	Front_intake_mg.move(0);
	Top_Intake.move(0);
	chassis.turnToHeading(-70, 800);
	Scraper.set_value(false);
	chassis.moveToPose(-54, 50, 0, 2000, {.lead = 0.7, .minSpeed = 90});
	chassis.moveToPose(-54, 40, 0, 2000, {.forwards = false});


	
	// SKILLS ____________________________
	
	// chassis.setPose(-48, 16, 90);
	// Descore.retract();
	// Front_intake_mg.move(127);
	// chassis.moveToPoint(-19, 23, 2000, {.maxSpeed=56}); //move to the first 4 balls
	// chassis.turnToHeading(320, 800);	
	// chassis.moveToPoint(-30, 43, 1000);
	// chassis.turnToPoint(-52, 48, 900); //aim for loading zone
	// chassis.waitUntilDone();
	// Scraper.set_value(true);
	// delay(100);
	// chassis.moveToPoint(-52, 48.7, 1000, {.minSpeed = 20}); //move to the loading 
	// chassis.waitUntilDone();
	// delay(1500);
	// chassis.moveToPoint(-13, 50.3, 1500, {.forwards = false, .maxSpeed = 70}); // score
	// chassis.waitUntilDone();
	// Front_intake_mg.move(127);
	// Top_Intake.move(127);
	// delay(3100);
	// Front_intake_mg.move(0);
	// Top_Intake.move(0);
	// Scraper.set_value(false);
	// chassis.moveToPose(-35, 26, 180, 1000, {.forwards=true, .lead = 0.2, .maxSpeed = 70});
	// chassis.waitUntilDone();
	// delay(200);
	// chassis.turnToHeading(90, 800);
	// Front_intake_mg.move(127);
	// chassis.waitUntilDone();
	// delay(200);
	// chassis.moveToPoint(28, 25, 1800, {.forwards=true, .maxSpeed = 100}); //move to 2nd stack
	// chassis.waitUntilDone();
	// delay(200);
	// chassis.moveToPose(48, 48.2, 45, 1400, {.forwards=true, .lead = 0.2, .maxSpeed = 50});
	// chassis.turnToHeading(90, 800);
	// chassis.waitUntilDone();
	// Scraper.set_value(true);
	// delay(600);
	// chassis.moveToPoint(66, 48, 800, {.forwards=true, .maxSpeed = 90}); //loading 2
	// chassis.waitUntilDone();
	// delay(1500);
	// chassis.moveToPoint(23, 49.2, 1800, {.forwards = false, .maxSpeed = 70}); // score
	// chassis.waitUntilDone();
	// Front_intake_mg.move(127);
	// Top_Intake.move(127);
	// delay(3100);
	// Front_intake_mg.move(0);
	// Top_Intake.move(0);
	// Scraper.set_value(false);
	// chassis.moveToPoint(35, 52, 800, {.minSpeed = 70}); 
	// chassis.turnToPoint(33, -19, 800);
	// chassis.moveToPoint(33, -19, 2000, {.maxSpeed = 90});
	// Front_intake_mg.move(127);
	// chassis.turnToPoint(52, -42.7, 800);
	// chassis.moveToPoint(52, -42.7, 800, {.forwards=true, .maxSpeed = 70});
	// chassis.turnToHeading(90, 800);
	// chassis.waitUntilDone();
	// Scraper.set_value(true);
	// delay(500);
	// chassis.moveToPoint(64, -44, 800, {.forwards=true});
	// chassis.waitUntilDone();
	// delay(2000);
	// chassis.moveToPoint(23, -46, 1000, {.forwards = false, .maxSpeed = 70}); // score
	// chassis.waitUntilDone();
	// Front_intake_mg.move(127);
	// Top_Intake.move(127);
	// delay(3100);
	// Front_intake_mg.move(0);
	// Top_Intake.move(0);
	// Scraper.set_value(false);
	// chassis.moveToPose(40, -23.5, 0, 1500, {.forwards=true, .lead = 0.2, .maxSpeed = 60});
	// chassis.waitUntilDone();
	// delay(500);
	// chassis.turnToHeading(-90, 800);
	// Front_intake_mg.move(127);
	// chassis.moveToPoint(-2, -23.5, 1500, {.forwards=true, .maxSpeed = 90});
	// chassis.moveToPose(-33, -44, 230, 1500, {.forwards=true, .lead = 0.3, .maxSpeed = 70});
	// chassis.turnToHeading(-90, 800);
	// chassis.waitUntilDone();
	// Scraper.set_value(true);
	// delay(600);
	// chassis.moveToPoint(-52, -45.8, 1500, {.forwards=true, .maxSpeed = 90});
	// Front_intake_mg.move(127);
	// chassis.waitUntilDone();
	// delay(1500);
	// Front_intake_mg.move(0);
	// chassis.moveToPoint(-5, -46, 1500, {.forwards=false, .maxSpeed = 70});
	// chassis.waitUntilDone();
	// Front_intake_mg.move(127);
	// Top_Intake.move(127);
	// delay(3100);
	// Front_intake_mg.move(0);
	// Top_Intake.move(0);
	// chassis.turnToHeading(-70, 800);
	// Scraper.set_value(false);
	// chassis.moveToPose(-63.5, 3, 0, 1800, {.lead = 0.78, .minSpeed = 50});
	


	

	// Scraper.set_value(false);
	// chassis.turnToHeading(0, 500);
	// chassis.moveToPoint(-10.5, 60, 1300, {.minSpeed=50}); // 2nd 4 ball stack
	// chassis.moveToPoint(-10.5, 85, 800, {.maxSpeed=50}); // 2nd 4 ball stack
	// chassis.moveToPoint(-29.5, 100, 2000, {.minSpeed=50});
	// chassis.waitUntilDone();
	// delay(100);
	// chassis.turnToHeading(0, 1000);
	// chassis.waitUntilDone();
	// Scraper.set_value(true);
	// delay(200);
	// chassis.moveToPoint(-34.8, 114, 1200, {.minSpeed=50}); //loading zone
	// chassis.waitUntilDone();
	// delay(2000);
	// chassis.moveToPoint(-35.1, 86, 2000, {.forwards = false}); //score
	// chassis.waitUntilDone();
	// Front_intake_mg.move(127);
	// delay(3200);
	// Front_intake_mg.move(0);
	// chassis.moveToPoint(-34, 107, 800);
	// chassis.turnToHeading(-255, 600);
	// chassis.moveToPoint(41, 81.5, 1600, {.minSpeed=40}); // 3rd 4 ball
	// Scraper.set_value(false);
	// chassis.turnToHeading(30, 500);
	// chassis.moveToPoint(65, 105, 1200, {.minSpeed=20});
	// chassis.turnToHeading(0, 400);
	// chassis.waitUntilDone();
	// Scraper.set_value(true);
	// delay(200);
	// chassis.moveToPoint(65.7, 119.5, 1000, {.minSpeed=50}); //loading zone
	// chassis.waitUntilDone();
	// delay(1500);
	// chassis.moveToPoint(66, 87, 1500, {.forwards = false}); 
	// chassis.waitUntilDone();
	// Front_intake_mg.move(127);
	// delay(3200);
	// Front_intake_mg.move(0);
	// chassis.moveToPose(40, 107, -90, 1800, {.forwards=true,.minSpeed=50});
	// chassis.turnToHeading(180, 600);
	// chassis.moveToPoint(40, 40, 1800, {.forwards=true,.minSpeed=70});
	// Scraper.set_value(false);
	// chassis.moveToPoint(69, 3, 1000);
	// chassis.turnToHeading(180, 400);
	// chassis.waitUntilDone();
	// Scraper.set_value(true);
	// delay(200);
	// chassis.moveToPoint(69, -11, 1000);
	// chassis.waitUntilDone();
	// delay(1500);
	// chassis.moveToPoint(69.5, 24.4, 1500, {.forwards = false}); 
	// chassis.waitUntilDone();
	// Front_intake_mg.move(127);
	// Top_Intake.move(127);
	// delay(3200);
	// Top_Intake.move(0);
	// Top_Intake.move(0);
	// chassis.moveToPoint(18, 14, 1500, {.forwards = true}); 
	// Scraper.set_value(false);
	// chassis.turnToHeading(180, 500);
	// chassis.moveToPoint(18, -30, 1500, {.forwards = true, .minSpeed = 80}); 

	// chassis.setPose(0, 0, 0);
	// chassis.moveToPoint(0, 40, 5000);


	// 7 solo awp
	// Aligner.extend();
	// chassis.setPose(-46, 12, 90);
	// Front_intake_mg.move(127);
	// Redirect_mg.move(127);
	// chassis.moveToPose(-24,24,45,200,{.forwards=true,.maxSpeed=90,.minSpeed=80});
	// chassis.moveToPose(-18,24,50,1000,{.forwards=true,.maxSpeed=90,.minSpeed=40});
	// chassis.waitUntilDone();
	// pros::delay(1000);

	// chassis.turnToHeading(315, 800);	
	
	// chassis.moveToPoint(-8, 11.5, 550, {.forwards=false,.minSpeed=30});
	// chassis.waitUntilDone();
	// Top_Intake.move(-100);
	// delay(1700);
	// chassis.moveToPose(-46, 46,270, 4000, {.forwards=true,.lead=0.35,.minSpeed=30,});
	// Aligner.retract();
	// Top_Intake.move(0);
	// chassis.waitUntilDone();
	// chassis.turnToHeading(270, 	350);
	// Scraper.set_value(true);
	// chassis.waitUntilDone();
	// chassis.cancelAllMotions();
	// Top_Intake.move(0);
	// Front_intake_mg.move(127);
	// chassis.arcade(45, 0);
	// delay(1400);

	// chassis.moveToPoint(-20, 48.5, 1000, {.forwards = false,.maxSpeed=100});
	// chassis.waitUntilDone();
	// Top_Intake.move(127);
	// delay(2000);

	// Scraper.retract();
	// Aligner.extend();


	// // comment this out if you want to stay at high goal (red half awp)

	// chassis.moveToPose(-22, -21, 180, 5000, {.lead=0.2, .minSpeed=10});
	// chassis.waitUntil(68);
	// Front_intake_mg.move(0);
	// chassis.waitUntilDone();

	// chassis.turnToPoint(-12, -4, 750);
	// chassis.waitUntilDone();
	// delay(200);
	
	// chassis.moveToPoint(0, 0, 1000, {.maxSpeed=60, .minSpeed=5});
	// chassis.waitUntil(6);
	// Front_intake_mg.move(-127);

	// chassis.cancelAllMotions();







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
	// Top_Intake.move(127); //score
	// Redirect_mg.move(127);
	// pros::delay(1700); //wait for scoring
	// Top_Intake.move(0);
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
	// Top_Intake.move(-100);
	// // Aligner.set_value(false);
	// Redirect_mg.move(127); 
	// pros::delay(1400);//score on midle goal
	

	
	// //split here for hgmg
	// Top_Intake.move(0);
	// chassis.moveToPoint(-21, 7, 750,{.minSpeed=100}); //location halfway to far blocks
	// chassis.moveToPoint(-23, -28, 1000,{.maxSpeed = 65}); //location of far blocks
	// Top_Intake.move(0);
	// chassis.moveToPose(-10,-14, 235, 1000, {.forwards = false, .minSpeed = 80}, true);
	/*chassis.turnToHeading(225, 700); //shoudl be 225
	chassis.waitUntilDone();
	// chassis.moveToPoint(-17, -16, 1000,{.forwards=false}); //location low goal*/
	// chassis.waitUntilDone();
	// Top_Intake.move(-90); //score on low goal

	// 7 ball left side
	// Aligner.set_value(false); //aligner extend
	// removeRed = false; //false for red side, true for blue side
	// chassis.setPose(-50, 14, 90);
	// Front_intake_mg.move(127);
	// Redirect_mg.move(127);
	// chassis.moveToPoint(-29,20,1000,{.maxSpeed=69}); //-30, 19
	// chassis.moveToPoint(-22.7, 25.7,750, {.maxSpeed = 40}); //collect the two blocks
	// // chassis.moveToPoint(-20, 24,200, {.maxSpeed = 45}); //collect the two blocks
	// chassis.turnToPoint(-50,49.5, 800,{.forwards = true, .minSpeed = 50});
	// Aligner.set_value(true); //aligner retract
	// Scraper.set_value(true); //put scraper down
	// chassis.moveToPoint(-47.5, 46, 1000, {.minSpeed=50}); //drive to loader
	// chassis.turnToHeading(270, 500);
	// chassis.moveToPoint(-64, 48.9, 1100, {.maxSpeed = 65, .minSpeed = 40}); //drive to loader
	// // pros::delay(1400);
	// pros::delay(650);

	// Redirect_mg.move(0);
	// Scraper.set_value(false);
	// Aligner.set_value(false);
	// chassis.moveToPoint(-26, 49.5, 1700, {.forwards = false, .maxSpeed = 50}); //go to hg
	// chassis.waitUntilDone();
	// Top_Intake.move(127); //score
	// Redirect_mg.move(127);
	// pros::delay(1700);
	
	// Top_Intake.move(0);
	// Redirect_mg.move(0);
	// chassis.moveToPoint(-45,51,800);
	// chassis.turnToHeading(59,800);
	// Descore.set_value(true);
	// chassis.moveToPoint(-16,62.5,2000,{.minSpeed = 50});
	



	// 7 ball 
	// Aligner.set_value(false); //aligner extend
	// removeRed = false; //false for red side, true for blue side
	// chassis.setPose(-50, 2.5, 90);
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
	// Front_intake_mg.move(0);
	// Redirect_mg.move(0);
	// Scraper.set_value(false);
	// Aligner.set_value(false);
	// chassis.moveToPoint(-28, -50, 1300, {.forwards = false, .maxSpeed = 80}); //go to hg
	// chassis.waitUntilDone();
	// Top_Intake.move(127); //score
	// Redirect_mg.move(127);
	// pros::delay(2000);
	// pros::delay(300);
	// Top_Intake.move(0);
	// Redirect_mg.move(0);
	// chassis.moveToPoint(-45,-51,800);
	// chassis.turnToHeading(32,800);
	// chassis.moveToPoint(-12,-12,1000,{.maxSpeed = 90});
	// chassis.waitUntilDone();
	// Front_intake_mg.move(-127);
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
		Scraper.set_value(false);
		pros::Task([&]{
			while(true){
				colorsort();
				std::cout << "running" << std::endl;
				delay(10);
			}
		});
		colorSorton = true;
	while (true) {
		// pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		//                  (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		//                  (pros ::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs
		// Arcade control scheme

		int dir = master.get_analog(ANALOG_LEFT_Y);  // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_LEFT_X);  // Gets the turn left/right from right joystick
		left_mg.move(dir + 0.70*turn);                      // Sets left motor voltage
		right_mg.move(dir - 0.70*turn);             	        // Sets right motor voltage
		removeRed = true;
		// if (master.get_digital(DIGITAL_L1)) {v
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
			Top_Intake.move(127);
			Front_intake_mg.move(127);
		}
		else if (master.get_digital(DIGITAL_R2)) {
			Top_Intake.move(90);
			Front_intake_mg.move(85);
			Trapdoor.extend();
		}
		else if (master.get_digital(DIGITAL_L1)) {
			Front_intake_mg.move(127);
		}
		else if (master.get_digital(DIGITAL_L2)) {
			Front_intake_mg.move(-127);
			
		}
		else {
			Front_intake_mg.move(0);
			Top_Intake.move(0);
			Trapdoor.retract();
		}

	
	 	
	 	if (master.get_digital_new_press(DIGITAL_Y)) {

		 	Descore.toggle();
		 }
	 	if(master.get_digital_new_press(DIGITAL_A)){
		 	Scraper.toggle();
	 	}

		pros::delay(25);                               // Run for 20 ms then update
	}
}