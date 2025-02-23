// This driver version is the base motor driver for all future modifications

// CHANGE LOG
  // Added distance sensing with US_100 UART mode
  // Changed numbering to begin from 0
  // Added Ultrasonic classes
  // Added bluetooth control
  // renumbered the motors to ensure that the left is 0

//  u_changelog
//  removed the -99 default assignments and made them zero due to unsigned int
//  combining code from ROSArduinoBridge
//  defined baudrate as #define BAUDRATE     57600
//  Included the definition of serial commands
//  changed the way to handle serial commands

//  3.0 changelog
  //  Modified pin numbers based on Mega sheild
  //  Just used tic and tac instead of w_tic and w_tac
  //  Introduced the currentMillis variable with tic
  //  Changed time variables to unsigned long
  //  Removed the mmDist array and corresponding return statements in the US_SensorClass file
  //  Optimized code to remove all warnings

//  3.1 changelog
  //  Changed the order of variables in MotorClass.h
  //  Added code for the emergency stop functionality
  //  Added a new command for the raspberry to know if the driver is in Active mode (A) or Emergency mode (E)

//  4.0 changelog
//    changed the normal go pushbutton to an rgb button
//    improved the emergency algorithm to account for debouncing
//    added a remote emergency stop functionality from the Pi


#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include "MotorClass.h"
#include "US_SensorClass.h"
#include "commands.h"
#include <SoftwareSerial.h>
#include "RGB_Button.h"


RGB_Button emergency_button(44, 45, 46, 43); // R, G, B, push_button pins

int startColor[3] = {255, 0, 0};  // Initial color: Red
int endColor[3] = {0, 255, 0};    // Target color: Green
int steps = 100;                  // Number of steps for the transition
int currentStep = 0;              // Current step of the transition
unsigned long RGBpreviousMillis = 0;
const long RGBinterval = 20; // Interval for color update in milliseconds
bool blinkState = false;          // Boolean to toggle the blink state
unsigned long blinkPreviousMillis = 0;
const long blinkInterval = 500;

#define BAUDRATE     57600
#define AUTO_STOP_INTERVAL 2000
#define USSensor_INTERVAL 500

const int US100_TX0 = 51, US100_RX0 = 50; // As Labeled on the sensor
const int US100_TX1 = 53, US100_RX1 = 52;

SoftwareSerial US100Serial0(US100_RX0, US100_TX0); // US100 US sensor 0 UART connection
SoftwareSerial US100Serial1(US100_RX1, US100_TX1); // US100 US sensor 1 UART connection

US_SensorClass USSensor[2] = {(US100Serial0), (US100Serial1)}; // Create US Sensor objects

int US_i = 0; // toggle sensors

int enc_A0 = 18, enc_B0 = 19, PWM0 = 4, dir0 = 30, brake0 = 32;

int enc_A1 = 20, enc_B1 = 21, PWM1 = 3, dir1 = 31, brake1 = 33;

Encoder encoder0(enc_A0, enc_B0);

MotorClass motor0(PWM0, dir0, brake0, true); // PWM, direction, brake, encforward

Encoder encoder1(enc_A1, enc_B1);

MotorClass motor1(PWM1, dir1, brake1, false); // PWM, direction, brake, encforward

bool emergencyStop = true;
const int GO_BT = 7;                       // The button that permits the robot to move it's motors after an emergency stop
const int NH = 28, NL = 29;  // Pins that receive input signals from two inputs, one is normally HIGH, the other is normally LOW

// Variables for debounce logic
bool lastNHState = HIGH;
bool lastNLState = LOW;
unsigned long lastDebounceTimeNH = 0;
unsigned long lastDebounceTimeNL = 0;
unsigned long debounceDelay = 50; 

const int POT_PIN_IN = A7;
float pot = 0, cmd = 0, target = 0, trgt_max = 200, dt, speed0 = 0, speed1 = 0, interval;
char ans;

//  ROS BRIDGE

/* Variable initialization */
bool usePID = true;

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Are we moving or not
bool moving = false;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char command;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

// last time we received a motor command
unsigned long tic, tac = 0, lastMotorCommand = AUTO_STOP_INTERVAL, lastUSSRead = -USSensor_INTERVAL;

/* Clear the current command parameters */
void resetCommand() {
  command = '\0';
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

void resetPID(){
   motor0.resetSpeed();
   motor1.resetSpeed();
}

/* Run a command.  Commands are defined in commands.h */
void runCommand() {
//  int i = 0;
//  char *p = argv1;
//  char *str;
//  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  
  switch(command) {
    case ANALOG_READ:
      Serial.println(analogRead(arg1));
      break;
    case GET_BAT:
      Serial.println("Not available at the moment");
      break;
    case DIGITAL_READ:
      Serial.println(digitalRead(arg1));
      break;
    case ANALOG_WRITE:
      analogWrite(arg1, arg2);
      Serial.println("OK"); 
      break;
    case DIGITAL_WRITE:
      digitalWrite(arg1, arg2);
      Serial.println("OK"); 
      break;
    case PIN_MODE:
      pinMode(arg1, arg2);
      Serial.println("OK");
      break;
    case ULTRA_SONIC:
      Serial.print(USSensor[LEFT].mmDist);
      Serial.print(" ");
      Serial.println(USSensor[RIGHT].mmDist);
      break;
    case READ_ENCODERS:
      Serial.print(motor0.forward_enc ? motor0.actual_pos : -motor0.actual_pos);
      Serial.print(" ");
      Serial.println(motor1.forward_enc ? motor1.actual_pos: -motor1.actual_pos);
      break;
     case RESET_ENCODERS:
      encoder0.write(0);
      encoder1.write(0);
      resetPID();
      Serial.println("OK");
      break;
    case MOTOR_BRAKE:
      motor1.brake_Motor();
      motor0.brake_Motor();
      Serial.println("OK");
      break;
    case RELEASE_BRAKE:
      motor1.release_brake();
      motor0.release_brake();
      Serial.println("OK");
      break;
    case MOTOR_SPEEDS:
      usePID = true;
  //    motor0.at_pos = true;
  //    motor1.at_pos = true;
      /* Reset the auto stop timer */
      lastMotorCommand = millis();
      moving = !(arg1 == 0 && arg2 == 0);
      setMotorSpeeds(arg1, arg2);
      Serial.println("OK"); 
      break;
    case MOTOR_RAW_PWM:
      usePID = false;
  //    motor0.at_pos = true;
  //    motor1.at_pos = true;
      /* Reset the auto stop timer */
      lastMotorCommand = millis();
      resetPID();
      moving = true; // Sneaky way to temporarily disable the PID - I'll do it another way.
      moving = !(arg1 == 0 && arg2 == 0);
      setMotorSpeeds(arg1, arg2);
      Serial.println("OK"); 
      break;
    case TEMP_READ:
  //    Temperatures of the motors
      Serial.println("Not available at the moment");
      break;
    case E_STATE:
      if (emergencyStop) Serial.println("E");
      else Serial.println("A");
      break;
    case E_STOP:
      emergencyStop = true;
      Serial.println("OK"); 
      break;
    case UPDATE_PID:
  //    while ((str = strtok_r(p, ":", &p)) != nullptr) {
  //       pid_args[i] = atoi(str);
  //       i++;
  //    }
  //    Kp = pid_args[0];
  //    Kd = pid_args[1];
  //    Ki = pid_args[2];
  //    Ko = pid_args[3];
      Serial.println("Not available at the moment");
      break;
    default:
      Serial.println("Invalid Command");
      break;
  }
}

// A convenience function for setting both motor speeds
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  speed0 = leftSpeed;
  speed1 = rightSpeed;
}

// END OF ROS BRIDGE


void setup() {
  Serial.begin(BAUDRATE); // Raspberry or workstation UART connection
  Serial2.begin(9600); // Bluetooth module UART connection
  pinMode(POT_PIN_IN, INPUT);
  pinMode(NH, INPUT); pinMode(NL, INPUT); // Emergency stuff
  motor1.init_Motor();
  motor0.init_Motor();
//  Serial.println("\n\n\nLET'S BEGIN\n\n\n\n");

  USSensor[US_i].initSensor(9600);

  emergency_button.init_RGBbutton();
  emergency_button.setColor(startColor);
  emergency_button.runColor();
}

void loop() {
  tic = millis(); // Get the current time
  
  // Get the current rotational positions
  motor0.current_pos = abs(motor0.actual_pos = encoder0.read()); 
  motor1.current_pos = abs(motor1.actual_pos = encoder1.read());

  interval = tic - tac; tac = tic; // Calculate elapsed time
  dt = interval / 1000.00; // Convert time to seconds

  // Get motor speeds in this time interval
  motor1.Speed(interval);
  motor0.Speed(interval);

  //  Get potentiometer reading
  pot = analogRead(POT_PIN_IN);
  pot = map(pot, 0, 1023, 0, trgt_max);

  // Check for emergency stop conditions
    // Read the state of the NH and NL pins
  bool currentNHState = digitalRead(NH);
  bool currentNLState = digitalRead(NL);


    // Debounce NH
  if (currentNHState != lastNHState) {
    lastDebounceTimeNH = tic;
  }
  if ((tic - lastDebounceTimeNH) > debounceDelay) {
    if (currentNHState == LOW) {
      emergencyStop = true;
    }
  }
  lastNHState = currentNHState;

  // Debounce NL
  if (currentNLState != lastNLState) {
    lastDebounceTimeNL = tic;
  }
  if ((tic - lastDebounceTimeNL) > debounceDelay) {
    if (currentNLState == HIGH) {
      emergencyStop = true;
    }
  }
  lastNLState = currentNLState;


//  // Reset emergency stop if button is pressed
//  if (digitalRead(GO_BT) == LOW) {
//    emergencyStop = false;
//  }
  
  //  NEW ROS BRIDGE
  while (Serial.available() > 0) {
    chr = Serial.read(); // Read the next character

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = '\0';
      else if (arg == 2) argv2[index] = '\0';
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = '\0';
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) command = chr; // The first arg is the single-letter command
      else if (arg == 1) argv1[index++] = chr;  // Subsequent arguments can be more than one character
      else if (arg == 2) argv2[index++] = chr;
    }
  }
  
  // Check to auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
    setMotorSpeeds(0, 0);
    moving = false;
  }

  //  Handle Bluetooth commands
  if (Serial2.available() > 0)
  {
    ans = Serial2.read(); // Read a single character
    
    switch (ans) {
      case 'f': motor1.rotNTimes(1); motor0.rotNTimes(1); break;
      case 'b': motor1.rotNTimes(-1); motor0.rotNTimes(-1); break;
      case 'l': motor1.rotNTimes(1); motor0.rotNTimes(-1); break;
      case 'r': motor1.rotNTimes(-1); motor0.rotNTimes(1); break;
      case 's': motor1.stop_Motor(); motor0.stop_Motor(); break;
      case 'h': motor1.brake_Motor(); motor0.brake_Motor(); break;
      case 'a': motor1.release_brake(); motor0.release_brake(); break;
      case 'm':
      motor1.max_vel += 10;
      motor0.max_vel += 10;
      motor1.max_vel = (motor1.max_vel > 100) ? 100 : motor1.max_vel;
      motor0.max_vel = (motor0.max_vel > 100) ? 100 : motor0.max_vel;
      break;
      case 'n':
      motor1.max_vel -= 10;
      motor0.max_vel -= 10;
      motor1.max_vel = (motor1.max_vel < 0) ? 0 : motor1.max_vel;
      motor0.max_vel = (motor0.max_vel < 0) ? 0 : motor0.max_vel;
      break;
      case 'v':
      motor1.max_vel = 50;
      motor0.max_vel = 50;
      break;
      default:
      break;
    }
  }


  //  Update Motor targets
  motor1.target_speed = cmd + pot;
  motor0.target_speed = cmd + pot;
  
  motor1.target_speed = (motor1.target_speed < 0) ? 0 : motor1.target_speed;
  motor0.target_speed = (motor0.target_speed < 0) ? 0 : motor0.target_speed;

  motor1.target_speed += speed1;
  motor0.target_speed += speed0;

  motor0.target_speed = constrain(motor0.target_speed, -trgt_max, trgt_max);
  motor1.target_speed = constrain(motor1.target_speed, -trgt_max, trgt_max);

  if (!motor1.at_pos) motor1.target_speed = motor1.rotTo();
  if (!motor0.at_pos) motor0.target_speed = motor0.rotTo();

  // motor1.PID(dt);
  // motor0.PID(dt);
  // motor1.pid = (motor1.pid < 20) ? (motor1.target_speed < 30) ? motor1.pid: 20 : motor1.pid;
  // motor0.pid = (motor0.pid < 20) ? (motor0.target_speed < 30) ? motor0.pid: 20 : motor0.pid;

  if (emergencyStop) {
    resetPID();
    motor0.target_speed = 0;
    motor1.target_speed = 0;
    if (tic - blinkPreviousMillis >= blinkInterval) {
      blinkPreviousMillis = tic;
      blinkState = !blinkState;
      if (blinkState) {
        emergency_button.setColor(255, 0, 0); // Red
      } else {
        emergency_button.setColor(0, 0, 0); // Off
      }
    }
  } else {
    // Handle normal color transitions
    if (tic - RGBpreviousMillis >= RGBinterval) {
      RGBpreviousMillis = tic;
      updateColor();
    }
  }

  if (emergency_button.buttonState()) {
    emergency_button.setColor((const int[]){255,255,255});
    emergencyStop = false;
  }

  
  
  emergency_button.runColor();
  
  // Run Motors
  if (usePID) {
    motor1.run_MotorPID(motor1.target_speed, dt);
    motor0.run_MotorPID(motor0.target_speed, dt);
  } else {
    motor1.run_Motor(motor1.target_speed);
    motor0.run_Motor(motor0.target_speed);
  }

  
//    Serial.print(motor0.target_speed); 
//    Serial.print(" "); 
//    Serial.println(motor1.target_speed); 
//    Serial.print(" "); 

  // Update US Sensors
  if ((tic - lastUSSRead) > USSensor_INTERVAL) {  // Check to see if we have exceeded the sensor cool down
    lastUSSRead = tic;
    USSensor[US_i].distance();
    if (US_i == 1) motor1.obstacle = (USSensor[1].mmDist < 100);
    else motor0.obstacle = (USSensor[0].mmDist < 100);
    
    US_i ^= 1;
    USSensor[US_i].initSensor(9600);
  }

//  Plotter(); 
}


void Plotter(){
  
  Serial.println(" ");
  Serial.print(0);            Serial.print("  ");                         // to limit plotter scale
  Serial.print(USSensor[0].mmDist);   Serial.print("  ");
  Serial.print(USSensor[1].mmDist);   Serial.print("  ");



   Serial.print(motor0.target_speed,3);   Serial.print("  ");
  Serial.print(motor0.current_pos);   Serial.print("  ");
  Serial.print(motor0.actual_pos);   Serial.print("  ");
  Serial.print(motor0.obstacle);   Serial.print("  ");
   Serial.print(motor0.pid,3);        Serial.print("  ");
   Serial.print(motor0.map_speed,3);   Serial.print("  ");
   Serial.print(motor0.error,3);        Serial.print("  ");
  // Serial.print(motor0.P,3);   Serial.print("  ");
  // Serial.print(motor0.I,3);   Serial.print("  ");
  // Serial.print(motor0.D,3);   Serial.print("  ");


   Serial.print(motor1.target_speed,3);   Serial.print("  ");
  Serial.print(motor1.current_pos);   Serial.print("  ");
  Serial.print(motor1.actual_pos);   Serial.print("  ");
  Serial.print(motor1.obstacle);   Serial.print("  ");
   Serial.print(motor1.pid,3);        Serial.print("  ");
   Serial.print(motor1.map_speed,3);   Serial.print("  ");
   Serial.print(motor1.error,3);        Serial.print("  ");
  // Serial.print(motor1.P,3);   Serial.print("  ");
  // Serial.print(motor1.I,3);   Serial.print("  ");
  // Serial.print(motor1.D,3);   Serial.print("  ");


  Serial.println(trgt_max,0);                                             // to limit plotter scale
}



void updateColor() {
  int color[3];
  for (int i = 0; i < 3; i++) {
    color[i] = startColor[i] + ((endColor[i] - startColor[i]) * currentStep) / steps;
  }
  emergency_button.setColor(color);
  currentStep++;

  if (currentStep > steps) {
    currentStep = 0;

    // Move to the next color transition
    for (int i = 0; i < 3; i++) {
      startColor[i] = endColor[i];
    }
    getNextColor(endColor);
  }
  
}

void getNextColor(int color[3]) {
  static int colorIndex = 0;
  int colors[][3] = {
    {255, 0, 0},   // Red
    {0, 255, 0},   // Green
    {0, 0, 255},   // Blue
    {255, 255, 0}, // Yellow
    {0, 255, 255}, // Cyan
    {255, 0, 255}, // Magenta
    {255, 255, 255} // White
  };

  for (int i = 0; i < 3; i++) {
    color[i] = colors[colorIndex][i];
  }

  colorIndex = (colorIndex + 1) % (sizeof(colors) / sizeof(colors[0]));
  
}
