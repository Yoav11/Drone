#include <Servo.h>

Servo L_F_prop;
Servo L_B_prop;
Servo R_F_prop;
Servo R_B_prop;

int rollError, pitchError;

int pitchDesiredAngle = 0;
int rollDesiredAngle = 0;



//////////////////////////////PID FOR ROLL///////////////////////////
float roll_PID, pwm_L_F, pwm_L_B, pwm_R_F, pwm_R_B, roll_error, roll_previous_error;
float roll_pid_p = 0;
float roll_pid_i = 0;
float roll_pid_d = 0;
///////////////////////////////ROLL PID CONSTANTS////////////////////
double roll_kp = 0.7; //3.55
double roll_ki = 0.006; //0.003
double roll_kd = 1.2; //2.05
float roll_desired_angle = 0;     //This is the angle in which we whant the

//////////////////////////////PID FOR PITCH//////////////////////////
float pitch_PID, pitch_error, pitch_previous_error;
float pitch_pid_p = 0;
float pitch_pid_i = 0;
float pitch_pid_d = 0;
///////////////////////////////PITCH PID CONSTANTS///////////////////
double pitch_kp = 0.72; //3.55
double pitch_ki = 0.006; //0.003
double pitch_kd = 1.22; //2.05
float pitch_desired_angle = 0;     //This is the angle in which we whant the

double input_THROTTLE = 1300; //initial value of throttle to the motors

void PIDSetup() {

  L_F_prop.attach(3); //left front motor
  L_B_prop.attach(11); //left back motor
  R_F_prop.attach(10); //right front motor
  R_B_prop.attach(9); //right back motor

  L_F_prop.writeMicroseconds(1000);
  L_B_prop.writeMicroseconds(1000);
  R_F_prop.writeMicroseconds(1000);
  R_B_prop.writeMicroseconds(1000);

}

void PIDControl() {

  pitchError = totalAngle[0] - pitchDesiredAngle;
  rollError = totalAngle[1] - rollDesiredAngle;

  roll_pid_p = roll_kp * roll_error;
  pitch_pid_p = pitch_kp * pitch_error;

  if (-3 < roll_error < 3) {
    roll_pid_i = roll_pid_i + (roll_ki * roll_error);
  }
  if (-3 < pitch_error < 3) {
    pitch_pid_i = pitch_pid_i + (pitch_ki * pitch_error);
  }

  roll_pid_d = roll_kd * ((roll_error - roll_previous_error) / elapsedTime);

  roll_PID = roll_pid_p + roll_pid_i + roll_pid_d;
  pitch_PID = pitch_pid_p + pitch_pid_i + pitch_pid_d;

  if (roll_PID < -400) {
    roll_PID = -400;
  }
  if (roll_PID > 400) {
    roll_PID = 400;
  }
  if (pitch_PID < -4000) {
    pitch_PID = -400;
  }
  if (pitch_PID > 400) {
    pitch_PID = 400;
  }

  pwm_R_F  = 115 + input_THROTTLE - roll_PID - pitch_PID;
  pwm_R_B  = 115 + input_THROTTLE - roll_PID + pitch_PID;
  pwm_L_B  = 115 + input_THROTTLE + roll_PID + pitch_PID;
  pwm_L_F  = 115 + input_THROTTLE + roll_PID - pitch_PID;



  if (pwm_R_F < 1100)
  {
    pwm_R_F = 1100;
  }
  if (pwm_R_F > 2000)
  {
    pwm_R_F = 2000;
  }

  //Left front
  if (pwm_L_F < 1100)
  {
    pwm_L_F = 1100;
  }
  if (pwm_L_F > 2000)
  {
    pwm_L_F = 2000;
  }

  //Right back
  if (pwm_R_B < 1100)
  {
    pwm_R_B = 1100;
  }
  if (pwm_R_B > 2000)
  {
    pwm_R_B = 2000;
  }

  //Left back
  if (pwm_L_B < 1100)
  {
    pwm_L_B = 1100;
  }
  if (pwm_L_B > 2000)
  {
    pwm_L_B = 2000;
  }

  roll_previous_error = roll_error; //Remember to store the previous error.
  pitch_previous_error = pitch_error; //Remember to store the previous error.

  L_F_prop.writeMicroseconds(pwm_L_F);
  L_B_prop.writeMicroseconds(pwm_L_B);
  R_F_prop.writeMicroseconds(pwm_R_F);
  R_B_prop.writeMicroseconds(pwm_R_B);


}
