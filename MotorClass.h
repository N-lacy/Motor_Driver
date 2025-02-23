#ifndef MOTORCLASS_H
#define MOTORCLASS_H

#include <Arduino.h>
#include "MegunoLink.h"
#include "Filter.h"


class MotorClass {
  private:
  int PWM_pin;
  int dir_pin;
  // int PWM;
  int brake_pin;
  bool brake = false;
  int max_speed = 200;
  int min_speed = 0;
  const int motor_steps = 3200, THRESH = 4;
  int dis;
  
  public:
  MotorClass(int pwm, int dir, int brk, bool enc_dir);
  void init_Motor();
  void run_Motor(float s);
  void run_MotorPID(float s, float t);
  void stop_Motor();
  void brake_Motor();
  void release_brake();
  void Speed(volatile float inter);
  void PID(float dt);
  void turnNTimes(float n);
  void rotNTimes(float n);
  float moveTo();
  float rotTo();
  void SetDirection(bool d);
  void resetSpeed();
  long actual_pos, current_pos, old_pos = 0, target_pos = 0; // Motor Positional variables
  int rot = 0, max_pwm = 255, speed_INDEX, max_vel = 50;
  volatile float speed_SUM = 0.0;
  static const int WINDOW_SIZE = 10;
  bool obstacle = false, forward_enc;
  // float speed_min = 3.2, speed_max = 7.2;
  float target_speed, speed, filtered_speed, smooth_speed, map_speed, speed_min = 0.0, speed_max = 14.2, speed_READINGS[WINDOW_SIZE] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Speed values
  bool direction = true, at_pos = true; // Rotation control variables
  float error=0.00, errDiff=0.00, prevErr=0.00, maxSum=150, errSum=0.00;  // PID variables
  float P=0.00, I=0.00, D=0.00, pid=0.00; // PID values
  float kp=0.5, ki=0.91, kd=0.02; // PID Coefs.
  // float kp=0.7, ki=0.9, kd=0.05; // PID Coefs.
  // float kp=1.51, ki=0.92, kd=0.061; // PID Coefs.


  ExponentialFilter<float> SpeedFilter;

};

#endif
