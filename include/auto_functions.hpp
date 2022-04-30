#ifndef _AUTO_FUNCTIONS_HPP_
#define _AUTO_FUNCTIONS_HPP_

#define FORWARD 1
#define BACKWARD 0

void drivePower(double forwardPower, double turningPower);
void benziDriveToPoint(quadratic_bezi_curve target, int direction, PIDConstants y_k, PIDConstants t_k, double velocity);
void driveToPoint(Vector2D target, int direction);
void areWeThereYet(Vector2D target, int direction, double timeOut, double accuracy);
void turn(double target, PIDConstants t_k, double timeout, double accuracy, double max);
void setArm_Height(double height);
void grabMogo();
void move(double target, PIDConstants y_k, PIDConstants t_k, double timeout, double accuracy, double max);
void grab_goal();
#endif