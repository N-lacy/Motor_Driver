//// Basically the same as the main code just minified
//#include <Arduino.h>
//#define ENCODER_OPTIMIZE_INTERRUPTS
//#include <Encoder.h>
//#include "MotorClass.h"
//#include "US_SensorClass.h"
//#include <SoftwareSerial.h>
//#define BAUDRATE 57600
//#define AUTO_STOP_INTERVAL 2000
//#define USSensor_INTERVAL 500
//#include "commands.h"
//
//const int US100_TX0 = 51, US100_RX0 = 50, US100_TX1 = 53, US100_RX1 = 52, enc_A0 = 18, enc_B0 = 19, PWM0 = 4, dir0 = 30, brake0 = 32, enc_A1 = 20, enc_B1 = 21, PWM1 = 3, dir1 = 31, brake1 = 33, POT_PIN_IN = A7;
//SoftwareSerial US100Serial0(US100_RX0, US100_TX0), US100Serial1(US100_RX1, US100_TX1);
//US_SensorClass USSensor[2] = {US100Serial0, US100Serial1};
//Encoder encoder0(enc_A0, enc_B0), encoder1(enc_A1, enc_B1);
//MotorClass motor0(PWM0, dir0, brake0, true), motor1(PWM1, dir1, brake1, false);
//
//float pot = 0, cmd = 0, trgt_max = 200, dt, speed0 = 0, speed1 = 0, interval;
//char ans, chr, command, argv1[16], argv2[16];
//long arg1, arg2;
//bool usePID = true, moving = false;
//int US_i = 0, arg = 0, index = 0;
//unsigned long tic, tac = 0, lastMotorCommand = AUTO_STOP_INTERVAL, lastUSSRead = -USSensor_INTERVAL;
//
//void resetCommand() { command = '\0'; memset(argv1, 0, sizeof(argv1)); memset(argv2, 0, sizeof(argv2)); arg = index = arg1 = arg2 = 0; }
//void resetPID() { motor0.resetSpeed(0.1); motor1.resetSpeed(0.1); }
//void setMotorSpeeds(int lSpeed, int rSpeed) { speed0 = lSpeed; speed1 = rSpeed; }
//void runCommand() {
//  arg1 = atoi(argv1); arg2 = atoi(argv2);
//  switch(command) {
//    case ANALOG_READ: Serial.println(analogRead(arg1)); break;
//    case GET_BAT: case TEMP_READ: case UPDATE_PID: Serial.println("Not available at the moment"); break;
//    case DIGITAL_READ: Serial.println(digitalRead(arg1)); break;
//    case ANALOG_WRITE: case DIGITAL_WRITE: case PIN_MODE: case MOTOR_BRAKE: case RELEASE_BRAKE: analogWrite(arg1, arg2); Serial.println("OK"); break;
//    case ULTRA_SONIC: Serial.print(USSensor[0].mmDist); Serial.print(" "); Serial.println(USSensor[1].mmDist); break;
//    case READ_ENCODERS: Serial.print(motor0.forward_enc ? motor0.actual_pos : -motor0.actual_pos); Serial.print(" "); Serial.println(motor1.forward_enc ? motor1.actual_pos: -motor1.actual_pos); break;
//    case RESET_ENCODERS: encoder0.write(0); encoder1.write(0); resetPID(); Serial.println("OK"); break;
//    case MOTOR_SPEEDS: usePID = true; lastMotorCommand = millis(); moving = !(arg1 == 0 && arg2 == 0); setMotorSpeeds(arg1, arg2); Serial.println("OK"); break;
//    case MOTOR_RAW_PWM: usePID = false; lastMotorCommand = millis(); resetPID(); moving = !(arg1 == 0 && arg2 == 0); setMotorSpeeds(arg1, arg2); Serial.println("OK"); break;
//    default: Serial.println("Invalid Command"); break;
//  }
//}
//
//void setup() { Serial.begin(BAUDRATE); Serial2.begin(9600); pinMode(POT_PIN_IN, INPUT); motor0.init_Motor(); motor1.init_Motor(); USSensor[US_i].initSensor(9600); }
//void loop() {
//  tic = millis(); interval = tic - tac; tac = tic; dt = interval / 1000.00;
//  motor0.current_pos = abs(motor0.actual_pos = encoder0.read()); motor1.current_pos = abs(motor1.actual_pos = encoder1.read());
//  motor0.Speed(interval); motor1.Speed(interval);
//  pot = map(analogRead(POT_PIN_IN), 0, 1023, 0, trgt_max);
//  
//  while (Serial.available() > 0) {
//    chr = Serial.read();
//    if (chr == 13) { if (arg == 1) argv1[index] = '\0'; else if (arg == 2) argv2[index] = '\0'; runCommand(); resetCommand(); }
//    else if (chr == ' ') { if (arg == 0) arg = 1; else if (arg == 1) { argv1[index] = '\0'; arg = 2; index = 0; } } 
//    else { if (arg == 0) command = chr; else if (arg == 1) argv1[index++] = chr; else if (arg == 2) argv2[index++] = chr; }
//  }
//
//  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) { setMotorSpeeds(0, 0); moving = false; }
//  
//  if (Serial2.available() > 0) {
//    ans = Serial2.read();
//    switch (ans) {
//      case 'f': motor0.rotNTimes(1); motor1.rotNTimes(1); break;
//      case 'b': motor0.rotNTimes(-1); motor1.rotNTimes(-1); break;
//      case 'l': motor0.rotNTimes(-1); motor1.rotNTimes(1); break;
//      case 'r': motor0.rotNTimes(1); motor1.rotNTimes(-1); break;
//      case 's': motor0.stop_Motor(); motor1.stop_Motor(); break;
//      case 'h': motor0.brake_Motor(); motor1.brake_Motor(); break;
//      case 'a': motor0.release_brake(); motor1.release_brake(); break;
//      case 'm': motor0.max_vel = min(100, motor0.max_vel + 10); motor1.max_vel = min(100, motor1.max_vel + 10); break;
//      case 'n': motor0.max_vel = max(0, motor0.max_vel - 10); motor1.max_vel = max(0, motor1.max_vel - 10); break;
//      case 'v': motor0.max_vel = motor1.max_vel = 50; break;
//      default: break;
//    }
//  }
//
//  motor0.target_speed = max(0, cmd + pot + speed0);
//  motor1.target_speed = max(0, cmd + pot + speed1);
//  motor0.target_speed = constrain(motor0.target_speed, -trgt_max, trgt_max);
//  motor1.target_speed = constrain(motor1.target_speed, -trgt_max, trgt_max);
//  if (!motor0.at_pos) motor0.target_speed = motor0.rotTo();
//  if (!motor1.at_pos) motor1.target_speed = motor1.rotTo();
//
//  if (usePID) { motor0.run_MotorPID(motor0.target_speed, dt); motor1.run_MotorPID(motor1.target_speed, dt); }
//  else { motor0.run_Motor(motor0.target_speed); motor1.run_Motor(motor1.target_speed); }
//
//  if ((tic - lastUSSRead) > USSensor_INTERVAL) { lastUSSRead = tic; USSensor[US_i].distance(); if (US_i == 1) motor1.obstacle = (USSensor[1].mmDist < 100); else motor0.obstacle = (USSensor[0].mmDist < 100); US_i ^= 1; USSensor[US_i].initSensor(9600); }
//}
