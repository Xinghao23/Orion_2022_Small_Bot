#include "main.h"
using namespace pros;

Motor leftFront(3);
Motor leftBack(2); 
Motor leftMid(18); 
Motor leftTop(19); 
Motor rightFront(13); 
Motor rightBack(11);
Motor rightMid(12);
Motor rightTop(14);
Motor converyer(1);
Motor leftArmMotor(15);
Motor rightArmMotor(16);
Controller master(E_CONTROLLER_MASTER);
ADIDigitalOut clamp(8);
ADIDigitalOut pull(7);
ADIDigitalOut claw(1);

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

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
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
void autonomous() {}

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
	int mogo_state = 0;
	bool claw_state = true; 
	double start_time = 0;
	
	while (true) {
		double rightY = master.get_analog(ANALOG_RIGHT_Y);
		double rightX = master.get_analog(ANALOG_RIGHT_X);
		double leftY = master.get_analog(ANALOG_LEFT_Y);

		leftFront = rightY + rightX;
		leftMid = rightY + rightX;
		leftBack = -rightY - rightX;
		leftTop =  -rightY - rightX;
		rightFront = -rightY + rightX;
		rightMid = -rightY + rightX;
		rightBack = rightY - rightX;
		rightTop = rightY - rightX;
		leftArmMotor = -leftY;
		rightArmMotor = leftY;
		if(master.get_digital(DIGITAL_L1) == 1){
			converyer = -127;
		}
		else if(master.get_digital(DIGITAL_L2) == 1){
			converyer = 127;
		}
		else{
			converyer = 0;
		}

		if(master.get_digital_new_press(DIGITAL_R1) == 1){
			claw_state = !claw_state;
			claw.set_value(claw_state);
		}
		
		if(master.get_digital_new_press(DIGITAL_R2) == 1){
			// pull.set_value(1);
			// clamp.set_value(0);
			start_time = millis();

			if(mogo_state == 1 || mogo_state == 0){
				mogo_state = 2;
			}
			else if (mogo_state == 2 || mogo_state == 3) {
				mogo_state = 0;
			}
		}

		switch (mogo_state) {
			case 0 :
			clamp.set_value(0);
			if ((millis() - start_time) > 200) mogo_state = 1;
			break;
			case 1 :
			pull.set_value(1);
			break;
			case 2 :
			pull.set_value(0);
			if ((millis() - start_time) > 200) mogo_state = 3;
			break;
			case 3 :
			clamp.set_value(1);
			break;
		}

	
	
	}
}
