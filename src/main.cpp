#include "main.h"
using namespace std;

Motor leftFront(3);
Motor leftBack(8); 
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
ADIDigitalOut clamper(8);
ADIDigitalOut pull(7);
ADIDigitalOut claw(1);
ADIDigitalIn frontMogoChecker(3);
ADIDigitalOut blocker(2);
ADIEncoder verticalTracking({10,3,4}, false);
ADIEncoder horizontalTracking({10,1,2},false);
Imu imu(5); 
Odom odom(&imu, &horizontalTracking, &verticalTracking);

void odom_task(void* param) {
	//*
	delay(50);
	printf("Initializing Odometry...\n");
	//* comp:
	odom.configure_starting(Vector2D(0,0), 0);
	odom.configure(4.3, 20, 4.3, 20, 50);
	//*/
	/* skills: 
	odom.configure_starting(Vector2D(0,0), 0);
	odom.configure(8.25, 20, 3.22, 20, 20);
	//*/
	printf("waiting for imu to initialize...\n");
	while (imu.is_calibrating() == true) {}
	printf("Initialization complete\n");

	while (true) {
		odom.calculate_position(ODOM_DEBUG_GLOBAL_POSITION);
	}
	//*/
}

Task task_odom (odom_task, NULL, TASK_PRIORITY_DEFAULT - 1, TASK_STACK_DEPTH_DEFAULT, "ODOM");



void initialize() {
	pros::lcd::initialize();



}


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
void competition_initialize() {
	claw.set_value(1);
	clamper.set_value(1);
}

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
	//blocker.set_value(1);
	
	grabMogo();
	// drivePower(50, 0);
	// delay(500);
	// drivePower(0,0);
	//areWeThereYet(Vector2D(0,100), FORWARD, 2000, 2);
	//while (true) {
	//	driveToPoint(Vector2D(0, 20));
	//	delay(20);
	//}
	
	PIDConstants y_k(3,0,8);
	PIDConstants t_k(2,0,8);
	areWeThereYet(Vector2D(0,0), BACKWARD, 10000,5);
	delay(300);
	blocker.set_value(0);
	setArm_Height(1700);
	delay(750);
	areWeThereYet(Vector2D(-25, -40), FORWARD, 10000, 5);
	delay(500);
	turn(-96, t_k, 3000, 1, 40);
	delay(500);
	move(-700, y_k, t_k, 5000, 5, 30);
	delay(500);
	grab_goal();
	converyer = -127;
	turn(-30, t_k, 3000, 1, 40);
	move(1200, y_k, t_k, 5000, 5, 40);
	turn(90, t_k, 3000, 1, 40);
	move(2000, y_k, t_k, 10000, 5, 100);
	/*
	quadratic_bezi_curve target;
	target.set_points(Vector2D(0,0), Vector2D(0, 100), Vector2D(100, 100));
	benziDriveToPoint(target, FORWARD, PIDConstants(10, 0, 30), PIDConstants(5, 0, 30), 100);
	target.set_points(Vector2D(100,100), Vector2D(0, 100), Vector2D(0, 0));
	benziDriveToPoint(target, BACKWARD, PIDConstants(10, 0, 30), PIDConstants(5, 0, 30), 100);
	*/
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
	int mogo_state = 0;
	bool claw_state = true; 
	bool blocker_state = true;
	double start_time = 0;
	//verticalTracking.reset(); 
	//horizontalTracking.reset();
	while (true) {
		lcd::print(0,"Your mom is: %f", odom.getPosition().x);
		lcd::print(1,"Your dad is: %f", odom.getPosition().y);
		double rightY = master.get_analog(ANALOG_RIGHT_Y);
		double rightX = master.get_analog(ANALOG_RIGHT_X);
		double leftY = master.get_analog(ANALOG_LEFT_Y);

		drivePower(rightY, rightX);
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
		if(master.get_digital_new_press(DIGITAL_A) == 1){
			blocker_state = !blocker_state;
			blocker.set_value(blocker_state);
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
			clamper.set_value(0);
			if ((millis() - start_time) > 100) mogo_state = 1;
			break;
			case 1 :
			pull.set_value(1);
			break;
			case 2 :
			pull.set_value(0);
			if ((millis() - start_time) > 100) mogo_state = 3;
			break;
			case 3 :
			clamper.set_value(1);
			break;
		}

	
	delay(20);
		
	}
}
