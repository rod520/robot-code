#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/motor_group.hpp"




#define STARTX -12
#define STARTY -59
#define STARTTHETA 90

// These are the x values for the goals, but also the block dispensers. 
// should be calibrated

#define LEFTGOALX -47

#define LEFTGOALY 47
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}


// *** our robot configuration ***
pros::MotorGroup left_motor_group({1}, pros::MotorGears::blue);
pros::MotorGroup right_motor_group({-2}, pros::MotorGears::blue);

// our belt and intake:
pros::Motor belt(3, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor intake(4, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);

lemlib::Drivetrain drivetrain(&left_motor_group, &right_motor_group, 9.25, 4.01, 200, 8);

pros::Imu imu(10);	

lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &imu);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(20, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              3 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(30, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              100, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              10 // maximum acceleration (slew)
);


/// *** driver config ***
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors,
                        &throttle_curve, 
                        &steer_curve
);









/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	chassis.calibrate();
	pros::lcd::initialize();
	pros::lcd::set_text(1, "this is our robot code");
	chassis.setPose(STARTX, STARTY, STARTTHETA);


	pros::lcd::register_btn1_cb(on_center_button);

	pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading

			pros::lcd::print(3, "left wheel: %f", left_motor_group.get_position());
			pros::lcd::print(4, "right wheel: %f", right_motor_group.get_position());
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
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

// our starting spot: as viewed from the side of the arena right of your side



/* more variable templates if we need it.
#define STARTX -46
#define STARTY -6.5
#define STARTTHETA 90

#define STARTX -46
#define STARTY -6.5
#define STARTTHETA 90
*/ 

void autonomous() {
    chassis.setPose(STARTX, STARTY, STARTTHETA);
    chassis.moveToPoint(-24, -50, 4000);
	while (chassis.isInMotion()){
		pros::delay(10);
	}
	chassis.moveToPoint(CLOSEGOALX, -48, 4000);


}
    // for pid tuning, later 
// chassis.turnToHeading(90, 100000);


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

	while (true){
		int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		chassis.curvature(leftY, rightX);
		
		
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			belt.move(-130);
			//outtake belt
			
		}
		else if	(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			belt.move(130);
			//intake belt
		}
		else{
			belt.brake();
		}
		if	(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			intake.move(60);
			// outtake intake
		}
		else if	(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			intake.move(-60);
			//intake intake
		}
		else{
			intake.brake();
		}

		pros::delay(25);
	}
}
