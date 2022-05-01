#include "main.h"

PID drive;
PID yPID;
PID turningPID;
void drivePower(double forwardPower, double turningPower){
    leftFront = forwardPower + turningPower;
	leftMid = forwardPower + turningPower;
	leftBack = -forwardPower - turningPower;
	leftTop =  -forwardPower - turningPower;
	rightFront = -forwardPower + turningPower;
	rightMid = -forwardPower + turningPower;
	rightBack = forwardPower - turningPower;
	rightTop = forwardPower - turningPower;
}


void benziDriveToPoint(quadratic_bezi_curve target, int direction, PIDConstants y_k, PIDConstants t_k, double velocity){
    PID y_PID;
    PID t_PID;

    y_PID.set_constants(y_k);
    y_PID.set_variables(0, 127, -127, 100);

    t_PID.set_constants(t_k);
    t_PID.set_variables(0, 127, -127, 100);
    
    double total_dist = 0;
    double segment_dist = 0;
    double current_dist = 0;
    double past_dist = 0;
    double counter = 0;
    double bezi_input = 0;

    while (counter < 200) {
        bezi_input = counter / 200.0;
        past_dist = current_dist;
        current_dist = target.getCurve(bezi_input).getLength();
        segment_dist = current_dist - past_dist;
        total_dist += segment_dist;
        counter++;
    }
    double runtime_sec = total_dist / velocity;
    double runtime_ms = runtime_sec * 1000;

    Timer runtime_timer;
    Timer exit_timer;

    double y_target = 0;
    double t_target = 0;
    Vector2D vectorTarget;
    Vector2D robotSpaceTarget;
    double angle = 0;

    while (runtime_timer.delta_time() < runtime_ms) {
        bezi_input = runtime_timer.delta_time() / runtime_ms;
        vectorTarget = target.getCurve(bezi_input);
        robotSpaceTarget = vectorTarget.getHeadingBased(odom.rad_angle());
        y_PID.set_target(robotSpaceTarget.y);

        if (direction == FORWARD) {
            angle = target.getFirstDerivative(bezi_input).getAngle() * 180 / 3.1415;
            t_PID.set_target(angle);
        }
        else {
            angle = target.getFirstDerivative(bezi_input).getAngle() * 180 / 3.1415;
            t_PID.set_target(angle - 180);
        }

        drivePower(y_PID.output(odom.getPosition().getHeadingBased(odom.rad_angle()).y), t_PID.output(odom.getAngle()));
    }

    drivePower(0,0);

}

void driveToPoint(Vector2D target, int direction){
    yPID.set_constants(3,0.001,8); //0,0,0
    turningPID.set_constants(5,0.001,8);
    Vector2D robotSpaceTarget = target.getHeadingBased(odom.rad_angle());
    Vector2D robotSpaceCurrentPosition = odom.getPosition().getHeadingBased(odom.rad_angle());
    yPID.set_variables(robotSpaceTarget.y, 100,-100, 2); 
    turningPID.set_variables(robotSpaceTarget.x, 100, -100, 2);
    if (direction == FORWARD) 
        drivePower(yPID.output(robotSpaceCurrentPosition.y), turningPID.output(robotSpaceCurrentPosition.x));
    else 
        drivePower(yPID.output(robotSpaceCurrentPosition.y), turningPID.output(-robotSpaceCurrentPosition.x));
}

void areWeThereYet(Vector2D target, int direction, double timeOut, double accuracy){
    Vector2D error = target - odom.getPosition();
    Timer timeOutTimer; 
    timeOutTimer.reset();

    while (error.getLength() > accuracy && timeOut >= timeOutTimer.delta_time()) {
        driveToPoint(target, direction);
        error = target - odom.getPosition(); 
        delay(20);
    }
    drivePower(0,0);
}
void grabMogo(){
    claw.set_value(0);
    double angle_error = 0;
    bool ready_for_goal = false;
    delay(20);
    blocker.set_value(1);
    while(fabs(odom.getPosition().y) < 130){
        if (ready_for_goal == false && frontMogoChecker.get_value() == 0) {
            ready_for_goal = true;
        }

        if (ready_for_goal == true && frontMogoChecker.get_value() == 1) {
            break;
        }
        angle_error = -(odom.getAngle());
        drivePower(127,angle_error * 4);
        delay(20);
    }
    drivePower(0,0);
    delay(20);
    claw.set_value(1);
    delay(200);
        
}

void turn(double target, PIDConstants t_k, double timeout, double accuracy, double max) {
    PID t_PID;
    t_PID.set_constants(t_k);
    t_PID.set_variables(target, max, -max, 2);
    double error = target - odom.getAngle();
    Timer timeout_timer;
    Timer exit_timer;
    exit_timer.reset();
    timeout_timer.reset();
    bool exit = false;

    while (exit == false && timeout_timer.delta_time() < timeout) {
        if (fabs(error) < accuracy) {
            if (exit_timer.delta_time() > 300) {
                exit = true;
            }
        }
        else {
            exit_timer.reset();
        }

        error = target - odom.getAngle();

        drivePower(0,t_PID.output(odom.getAngle()));
    }

    drivePower(0,0);
    
}

void setArm_Height(double height) {
    leftArmMotor.move_absolute(-height, 100);
    rightArmMotor.move_absolute(height, 100);
}

void move(double target, PIDConstants y_k, PIDConstants t_k, double timeout, double accuracy, double max) {
    PID t_PID;
    PID y_PID;
    t_PID.set_constants(t_k);
    t_PID.set_variables(odom.getAngle(), max, -max, 2);
    y_PID.set_constants(y_k);
    y_PID.set_variables(target, max, -max, 2);
    leftMid.tare_position();
    double error = target - leftMid.get_position();
    Timer timeout_timer;
    Timer exit_timer;
    exit_timer.reset();
    timeout_timer.reset();
    bool exit = false;

    while (exit == false && timeout_timer.delta_time() < timeout) {
        if (fabs(error) < accuracy) {
            if (exit_timer.delta_time() > 300) {
                exit = true;
            }
        }
        else {
            exit_timer.reset();
        }

        error = target - leftMid.get_position();

        drivePower(y_PID.output(leftMid.get_position()),t_PID.output(odom.getAngle()));
    }

    drivePower(0,0);
}

void grab_goal() {
    clamper.set_value(0);
    delay(300);
    pull.set_value(1);
}

void move_to_point(Vector2D target, int direction, double accuracy, double max, double timeout, PIDConstants y_k, PIDConstants t_k) {
    Vector2D error = target - odom.getPosition();
    Vector2D local_current = odom.getPosition().getHeadingBased(odom.rad_angle());
    Vector2D local_target = target.getHeadingBased(odom.rad_angle());
    bool exit = false;
    Timer exit_timer;
    Timer timeout_timer;

    PID yPID;
    yPID.set_constants(y_k);
    yPID.set_variables(local_target.y, max, -max, 15);

    PID tPID;
    tPID.set_constants(t_k);
    tPID.set_variables(local_target.x, max, -max, 2);

    while (exit == false && timeout_timer.delta_time() < timeout) {
        error = target - odom.getPosition();
        local_current = odom.getPosition().getHeadingBased(odom.rad_angle());
        local_target = target.getHeadingBased(odom.rad_angle());

        yPID.set_target(local_target.y);
        tPID.set_target(local_target.x);

        if (direction == FORWARD)
            drivePower(yPID.output(local_current.y), tPID.output(local_current.x));
        else 
            drivePower(yPID.output(local_current.y), tPID.output(local_current.x) * -1);

        if (error.getLength() < accuracy) {
            if (exit_timer.delta_time() > 500) {
                exit = true;
            }
        }
        else {
            exit_timer.reset();
        }

        delay(20);
    }

    drivePower(0,0);
}

/*void driveForward(double target){
    set_constants(1,1,1);
    drive.set_target(target);    
    set_variables(target, 127,-127, 100);
    while(motor.get_position() <= target){
        drivePower(drive.output(motor.get_position(),0);
    }
    drivePower(0,0);
}*/
