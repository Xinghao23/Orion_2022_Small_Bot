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
void benziDriveToPoint(quadratic_bezi_curve target, PIDConstants y_k, PIDConstants t_k, double velocity){
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

    while (runtime_timer.delta_time() < runtime_ms) {
        bezi_input = runtime_timer.delta_time() / runtime_ms;
        vectorTarget = target.getCurve(bezi_input);
        driveToPoint(vectorTarget); 
        
    }

    drivePower(0,0);

}
void driveToPoint(Vector2D target){
    yPID.set_constant(0,0,0); 
    turningPID.set_constants(0,0,0);
    Vector2D robotSpaceTarget = target.getHeadingBased(odom.rad_angle());
    Vector2D robotSpaceCurrentPosition = odom.getPosition().getHeadingBased(odom.rad_angle());
    yPID.set_variables(robotSpaceTarget.y, 100,-100, 100); 
    turningPID.set_variables(robotSpaceTarget.x, 100, -100, 100);
    drivePower(yPID.output(robotSpaceCurrentPosition.y), turningPID.output(robotSpaceCurrentPosition.x));
}

void areWeThereYet(Vector2D target, double timeOut, double accuracy){
    Vector2D error = target - odom.getPosition();
    Timer timeOutTimer; 
    timeOutTimer.reset();

    while (error.getLength() > accuracy || timeOut >= timeOutTimer.delta_time()) {
        driveToPoint(target);
        error = target - odom.getPosition(); 
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
