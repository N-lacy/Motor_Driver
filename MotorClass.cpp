#include "MotorClass.h"


MotorClass::MotorClass(int pwm, int dir, int brk, bool enc_dir)
  : PWM_pin(pwm), dir_pin(dir), brake_pin(brk), forward_enc(enc_dir), SpeedFilter(10, 0) {}


void MotorClass::init_Motor() {
  // Serial.println("Initialising");
  pinMode(dir_pin, OUTPUT);
  digitalWrite(dir_pin, HIGH);
  pinMode(brake_pin, OUTPUT);
  digitalWrite(brake_pin, brake);
  pinMode(PWM_pin, OUTPUT);
  analogWrite(PWM_pin, 0);
}


void MotorClass::run_Motor(float s) {

  if (obstacle) s = 0;
  
  if (s >= 0) {
    // Serial.println("Possitive rotation");
    if (!direction) {
      // Serial.println("Setting HIGH");
      direction = true;
      digitalWrite(dir_pin, HIGH);
    }
  }
  else {
    // Serial.println("Negative rotation");
    if (direction) {
      // Serial.println("Setting LOW");
      direction = false;
      digitalWrite(dir_pin, LOW);
    }
  }
  
  if (abs(s) != 0) release_brake();
  analogWrite(PWM_pin, abs(s));
}


void MotorClass::run_MotorPID(float s, float t) {

  if (obstacle) s = 0;

  if (s >= 0) {
    // Serial.println("Possitive rotation");
    if (!direction) {
      // Serial.println("Setting HIGH");
      direction = true;
      digitalWrite(dir_pin, HIGH);
    }
  }
  else {
    // Serial.println("Negative rotation");
    if (direction) {
      // Serial.println("Setting LOW");
      direction = false;
      digitalWrite(dir_pin, LOW);
    }
  }

  target_speed = abs(s);
  
  PID(t);
  
  pid = (pid < 20) ? (target_speed < 30) ? pid: 20 : pid;
  
  if (pid != 0 && brake) release_brake();
  analogWrite(PWM_pin, abs(pid));
}


void MotorClass::SetDirection(bool d) {
  direction = d;
}


void MotorClass::Speed(volatile float inter) {
  
  rot = abs(current_pos - old_pos); // Calcutate rotated angle
  old_pos = current_pos;
  
  speed = (inter > 0) ? rot / inter : 0; // Calculate speed

  SpeedFilter.Filter(speed); // Filter speed
  filtered_speed = SpeedFilter.Current();

  speed_READINGS[speed_INDEX] = filtered_speed; // Further filter speed with running average
  speed_SUM = 0.0;
  for (auto speeds: speed_READINGS)
    speed_SUM += isnan(speeds) ? 0 : speeds;
  smooth_speed = speed_SUM / WINDOW_SIZE;
  speed_INDEX = (speed_INDEX + 1) % WINDOW_SIZE;

  map_speed = map(smooth_speed * 100000, speed_min * 100000, speed_max * 100000, 0, max_pwm); // FIX THE MAPPING

}


void MotorClass::PID(float dt) {
  if (!(target_speed == 0 && abs(errSum) < 0.001)) {
    error = target_speed - map_speed;
    P = kp * error;
    I = ki * (errSum = errSum + (error * dt)); errSum = constrain(errSum, -maxSum, maxSum); errSum += (target_speed == 0) ? -0.5 * errSum : 0;   
    D = kd * (error - prevErr) / dt;  prevErr = error;
    pid = round(constrain(P + I + D + target_speed, 0, max_pwm));
  } else pid = 0.0;
}


void MotorClass::resetSpeed() {
  error = 0;
//  P = 0;
//  I = ki * (errSum = errSum + (error * dt)); errSum = 0;   
//  D = kd * (error - prevErr) / dt;  prevErr = error;
  pid = 0;
  
  P = 0;
  I = 0; errSum = 0;   
  D = 0;  prevErr = 0;
}


void MotorClass::turnNTimes(float n) {
  if (direction) target_pos = round(current_pos + motor_steps * n);
  else target_pos = round(current_pos - motor_steps * n);
  // target_pos = round(current_pos + motor_steps * n);
  at_pos = false;
}


float MotorClass::moveTo() {
  if (target_pos > current_pos)
  {
    dis = target_pos - current_pos;
    at_pos = false;
    if (dis > 2200) return 0.7 * constrain(map(dis, 3200, 3200 * 4, 25, 50), 30, max_vel);
    else return 0.7 * constrain(map(dis, 3200, 3200 * 4, 25, 50), 5, max_vel);
  } else {
    at_pos = true;
    return 0;
  }
}


void MotorClass::rotNTimes(float n) {
  if (forward_enc) {
    target_pos = round(actual_pos + motor_steps * n);
  } else {
    target_pos = round(actual_pos - motor_steps * n);
  }
  // if (current_pos < target_pos) direction = true;
  // else direction = false;
  at_pos = false;
}


float MotorClass::rotTo() {
  dis = target_pos - actual_pos;
  if (forward_enc) {
    if (dis > THRESH)
    {
      if (dis > 2200) return 0.7 * constrain(map(abs(dis), 2200, 3200 * 4, 25, max_vel), 30, max_vel);
      else return 0.8 * constrain(map(abs(dis), 2200, 3200 * 4, 25, max_vel), 5, max_vel);
    } else if (dis < -THRESH) {
      if (dis < -2200) return -0.7 * constrain(map(abs(dis), 2200, 3200 * 4, 25, max_vel), 30, max_vel);
      else return -0.8 * constrain(map(abs(dis), 2200, 3200 * 4, 25, max_vel), 5, max_vel);
    } else {
      at_pos = true;
      return 0;
    } 

  } else {
    if (dis < -THRESH)
    {
      if (dis < -2200) return 0.7 * constrain(map(abs(dis), 2200, 3200 * 4, 25, max_vel), 30, max_vel);
      // if (dis < -2200) return 30;
      else return 0.8 * constrain(map(abs(dis), 2200, 3200 * 4, 25, max_vel), 5, max_vel);
    } else if (dis > THRESH) {
      if (dis > 2200) return -0.7 * constrain(map(abs(dis), 2200, 3200 * 4, 25, max_vel), 30, max_vel);
      // if (dis > 2200) return -30;
      else return -0.8 * constrain(map(abs(dis), 2200, 3200 * 4, 25, max_vel), 5, max_vel);
    } else {
      at_pos = true;
      return 0;
    } 
  }
}


void MotorClass::stop_Motor() {
  // do {
    run_MotorPID(0,0.2);
    for (int i = 0; i < WINDOW_SIZE; i++){
      SpeedFilter.Filter(0); // Filter speed
      filtered_speed = SpeedFilter.Current();
      speed_READINGS[i] = 0;
      resetSpeed();
    } 
    run_MotorPID(0,0.2);

    analogWrite(PWM_pin, 0);
    at_pos = true;
  // } while (Motor_speed() > 0);
  
}


void MotorClass::brake_Motor() {
  stop_Motor();
  brake = true;
  digitalWrite(brake_pin, brake);
}


void MotorClass::release_brake() {
  brake = false;
  digitalWrite(brake_pin, brake);
}
