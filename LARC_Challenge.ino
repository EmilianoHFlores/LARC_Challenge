#include <Arduino.h>
#include <math.h>
#include "Motor.h"
#include "LARC_Base.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//Comment to use R0
#define R1

bool CHECK_MOTORS = false;
bool CHECK_ENCODERS = false;
bool CHECK_LINES = false;
bool CHECK_BNO = false;
bool CHECK_ENCODERS_100ms = true;
bool MOTOR_PID = true;

unsigned long time;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
#define BNO055_SAMPLERATE_DELAY_MS (100)
#define PID_SAMPLE_RATE 50

// END: BNO

//LINE SENSORS
static constexpr uint8_t kLineFrontLines[3] = {A7, A8, A9};
static constexpr uint8_t kLineBackLines[3] = {A4, A5, A6};
static constexpr uint8_t kLineLeftLines[3] = {A13, A14, A15};
static constexpr uint8_t kLineRightLines[3] = {A10, A11, A12};

//white tresholds
uint16_t frontLine0_ts = 0;
uint16_t frontLine1_ts = 0;
uint16_t frontLine2_ts = 0;
uint16_t backLine0_ts = 0;
uint16_t backLine1_ts = 0;
uint16_t backLine2_ts = 0;
uint16_t leftLine0_ts = 0;
uint16_t leftLine1_ts = 0;
uint16_t leftLine2_ts = 0;
uint16_t rightLine0_ts = 0;
uint16_t rightLine1_ts = 0;
uint16_t rightLine2_ts = 0;

//treshold to see white
const uint16_t linets = 200;

// START: MOTORS

// FRONT LEFT PINOUT
static constexpr uint8_t kDigitalPinsFrontLeftMotor[2] = {49, 48};
static constexpr uint8_t kAnalogPinFrontLeftMotor = 7;
static constexpr uint8_t kEncoderPinsFrontLeftMotor[2] = {2, 5};
// BACK LEFT PINOUT
#ifdef R1
static constexpr uint8_t kDigitalPinsBackLeftMotor[2] = {50, 51};
static constexpr uint8_t kAnalogPinBackLeftMotor = 6;
static constexpr uint8_t kEncoderPinsBackLeftMotor[2] = {3, 4};
#else
static constexpr uint8_t kDigitalPinsBackLeftMotor[2] = {32, 33};
static constexpr uint8_t kAnalogPinBackLeftMotor = 6;
static constexpr uint8_t kEncoderPinsBackLeftMotor[2] = {3, 4};
#endif
// FRONT RIGHT PINOUT
static constexpr uint8_t kDigitalPinsFrontRightMotor[2] = {47, 46};
static constexpr uint8_t kAnalogPinFrontRightMotor = 8;
static constexpr uint8_t kEncoderPinsFrontRightMotor[2] = {18, 17};
// BACK RIGHT PINOUT
#ifdef R1
static constexpr uint8_t kDigitalPinsBackRightMotor[2] = {44,45};
static constexpr uint8_t kAnalogPinBackRightMotor = 9;
static constexpr uint8_t kEncoderPinsBackRightMotor[2] = {19, 16};
#else
static constexpr uint8_t kDigitalPinsBackRightMotor[2] = {43,42};
static constexpr uint8_t kAnalogPinBackRightMotor = 9;
static constexpr uint8_t kEncoderPinsBackRightMotor[2] = {19, 16};
#endif

// ID: 0 = Back Left, 1 = Front Left, 2 = Back Right, 3 = Front Right
// DIRECTION: -1 = Backward, 0 = Stop, 1 = Forward
// PWM: 0-255
#define BACK_LEFT 0
#define FRONT_LEFT 1
#define BACK_RIGHT 2
#define FRONT_RIGHT 3
#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3


#ifdef R1
const double kp = 150;
const double ki = 50;
const double kd = 50;
#else
const double kp = 200;
const double ki = 75;
const double kd = 0;
#endif

const double anglekP = 9;
const double anglekI = 3;
const double anglekD = 5;

double error_sum = 0;
double error_prev = 0;

//encoders
static constexpr double kPulsesPerRevolution = 450.0;
const int front_left_dir_sign = 1;
int front_left_encoder = 0;
int front_left_dir = 0;

const int front_right_dir_sign = 1;
int front_right_encoder = 0;
int front_right_dir = 0;

const int back_left_dir_sign = 1;
int back_left_encoder = 0;
int back_left_dir = 0;

const int back_right_dir_sign = -1;
int back_right_encoder = 0;
int back_right_dir = 0;

double max_error_sum = 250;
double pwm_min = 80;
double pwm_max = 255;

Motor front_left_motor(1, kDigitalPinsFrontLeftMotor[0], kDigitalPinsFrontLeftMotor[1], kAnalogPinFrontLeftMotor, kEncoderPinsFrontLeftMotor[0], kEncoderPinsFrontLeftMotor[1]);
Motor front_right_motor(2, kDigitalPinsFrontRightMotor[0], kDigitalPinsFrontRightMotor[1], kAnalogPinFrontRightMotor, kEncoderPinsFrontRightMotor[0], kEncoderPinsFrontRightMotor[1]);
Motor back_left_motor(3, kDigitalPinsBackLeftMotor[0], kDigitalPinsBackLeftMotor[1], kAnalogPinBackLeftMotor, kEncoderPinsBackLeftMotor[0], kEncoderPinsBackLeftMotor[1]);
Motor back_right_motor(4, kDigitalPinsBackRightMotor[0], kDigitalPinsBackRightMotor[1], kAnalogPinBackRightMotor, kEncoderPinsBackRightMotor[0], kEncoderPinsBackRightMotor[1]);

void frontLeftEncoderCallback() {
  //front_left_dir = (int)(digitalRead(kEncoderPinsFrontLeftMotor[1]) == HIGH ? front_left_dir_sign : front_left_dir_sign * -1);
  front_left_encoder++;
  front_left_motor.updatePIDTicks();
}
void frontRightEncoderCallback() {
  //front_right_dir = (int)(digitalRead(kEncoderPinsFrontRightMotor[1]) == HIGH ? front_right_dir_sign : front_right_dir_sign * -1);
  front_right_encoder++;
  front_right_motor.updatePIDTicks();
}
void backLeftEncoderCallback() {
  //back_left_dir = (int)(digitalRead(kEncoderPinsBackLeftMotor[1]) == HIGH ? back_left_dir_sign : back_left_dir_sign * -1);
  back_left_encoder++;
    back_left_motor.updatePIDTicks();
}
void backRightEncoderCallback() {
  //back_right_dir = (int)(digitalRead(kEncoderPinsBackRightMotor[1]) == HIGH ? back_right_dir_sign : back_right_dir_sign * -1);
  back_right_encoder++;
  back_right_motor.updatePIDTicks();
}
void initEncoders() {
  attachInterrupt(digitalPinToInterrupt(kEncoderPinsFrontLeftMotor[0]), frontLeftEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(kEncoderPinsFrontRightMotor[0]), frontRightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(kEncoderPinsBackLeftMotor[0]), backLeftEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(kEncoderPinsBackRightMotor[0]), backRightEncoderCallback, RISING);
}

LARC_Base base = LARC_Base(&front_left_motor, &front_right_motor, &back_left_motor, &back_right_motor, &bno);
bool logLines = false;
bool logLLines = false;
bool logRLines = false;

//According to the challenge
int initialAngle = 90;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);  // wait for serial port to open!
    base.init();
    base.setPID(kp, ki, kd, PID_SAMPLE_RATE);
    base.setAnglePID(anglekP, anglekI, anglekD, PID_SAMPLE_RATE);
    base.setLogPID(false);
    base.setLogAnglePID(true);
    logLines = false;
    logLLines = false;
    logRLines = false;
    initEncoders();
    Serial.println("Calibrating lines");
    calibrateLines(50);
    Serial.println("Reorienting");
    initialReorientation(initialAngle);
    Serial.println("Done, Starting...");
    base.changeDirection(STOP);
}
// Used if ROS disabled.
int moveRPM = 275;
void loop() {
  //base.goForward(250);
  //From 6,7 to 2,4
  advanceForwardBNO(moveRPM, 4);
  delay(250);
  base.reorientate(-90);
  delay(250);
  advanceForwardBNO(moveRPM, 3);
  while(1){}
    /*advanceForwardBNO(moveRPM, 5);
  Serial.println("STOPPING");
  delay(250);
  base.reorientate(-90);
  delay(250);
  advanceForwardBNO(moveRPM, 2);
  Serial.println("STOPPING");
  while(1){}*/
  /*advanceForwardBNO(250, 5);
  Serial.println("STOPPING");
  delay(1000);
  base.reorientate(-90);
  delay(500);
  advanceForwardBNO(250, 2);
  Serial.println("STOPPING");
  while(1){}*/
  /*advanceForward(250,2);
  Serial.println("STOPPING");
  delay(1000);
  base.reorientate(90);
  delay(500);
  advanceForward(250, 2);
  Serial.println("STOPPING");
  while(1){}*/
  /*time = millis();
  while (millis() - time < 1000) {
    base.mecanumLeft();
    base.baseConstantRPM(250);
  }
  Serial.println("STOPPING");
    base.changeDirection(STOP);
    delay(1000);
  time = millis();
  while (millis() - time < 1000) {
    base.mecanumRight();
    base.baseConstantRPM(250);
  }
  base.changeDirection(STOP);
    while(1){};*/
  /*Serial.print("Front white: ");
  Serial.print(frontWhite());
  Serial.print(" Back white: ");
  Serial.println(backWhite());
  delay(100);*/
  /*
  base.reorientate(0);
  while(1){}*/
  /*
  time = millis();
  while (millis() - time < 2500) {
    base.changeDirection(FORWARD);
    base.baseConstantRPM(250);
  }
  Serial.println("STOPPING");
    base.changeDirection(STOP);
    time = millis();
  while (millis() - time < 2500) {
    base.changeDirection(BACKWARD);
    base.baseConstantRPM(250);
  }
    while(1){}*/
}

void initialReorientation(int angle){
  base.reorientate(-angle);
  base.resetBNO();
}

void advanceForwardBNO(double speed, int tiles){
  base.goForward(speed);
  bool repositionLeft = false;
  bool repositionRight = false;
  for (int i=0; i<tiles; i++){
    while(frontWhite()){
      base.goForward(speed);
    }
    while(!frontWhite()){
      base.goForward(speed);
    }
    Serial.println("Jumped line");
  }

  //Trust current pwm for last square
  while(backWhite()){
    base.goForward(speed);
  }
  base.changeDirection(BRAKE);
}

void advanceBackwardsBNO(double speed, int tiles){
  base.goBackwards(speed);
  bool repositionLeft = false;
  bool repositionRight = false;
  for (int i=0; i<tiles; i++){
    while(backWhite()){
      base.goBackwards(speed);
    }
    while(!backWhite()){
      base.goBackwards(speed);
    }
    Serial.println("Jumped line");
  }

  //Trust current pwm for last square
  while(frontWhite()){
    base.goBackwards(speed);
  }
  base.changeDirection(BRAKE);
}

void advanceForward(double speed, int tiles){
  base.changeDirection(FORWARD);
  base.baseConstantRPM(speed);
  bool repositionLeft = false;
  bool repositionRight = false;
  for (int i=0; i<tiles; i++){
    while(frontWhite()){
      base.baseConstantRPM(speed);
    }
    while(!frontWhite()){
      base.baseConstantRPM(speed);
    }
    Serial.println("Jumped line");
  }

  //Trust current pwm for last square
  while(backWhite()){
    base.baseConstantRPM(speed);
  }
  base.changeDirection(BRAKE);
}

void advanceBackwards(double speed, int tiles){
  base.changeDirection(BACKWARD);
  base.baseConstantRPM(speed);
  bool repositionLeft = false;
  bool repositionRight = false;
  for (int i=0; i<tiles; i++){
    while(backWhite()){
      base.baseConstantRPM(speed);
    }
    while(!backWhite()){
      base.baseConstantRPM(speed);
    }
    Serial.println("Jumped line");
  }

  //Trust current pwm for last square
  while(frontWhite()){
    base.baseConstantRPM(speed);
  }
  base.changeDirection(BRAKE);
  if (repositionLeft){
    Serial.println("Repositioning left");
  }
  if (repositionRight){
    Serial.println("Repositioning right");
  }
}

void calibrateLines(int n){
  //tresholds
  delay(1000);
  frontLine0_ts = 0;
  frontLine1_ts = 0;
  frontLine2_ts = 0;
  backLine0_ts = 0;
  backLine1_ts = 0;
  backLine2_ts = 0;
  leftLine0_ts = 0;
  leftLine1_ts = 0;
  leftLine2_ts = 0;
  rightLine0_ts = 0;
  rightLine1_ts = 0;
  rightLine2_ts = 0;
  for (int i=0; i<n; i++){
    //Reading white values
    frontLine0_ts += analogRead(kLineFrontLines[0]);
    frontLine1_ts += analogRead(kLineFrontLines[1]);
    frontLine2_ts += analogRead(kLineFrontLines[2]);
    backLine0_ts += analogRead(kLineBackLines[0]);
    backLine1_ts += analogRead(kLineBackLines[1]);
    backLine2_ts += analogRead(kLineBackLines[2]);
    leftLine0_ts += analogRead(kLineLeftLines[0]);
    leftLine1_ts += analogRead(kLineLeftLines[1]);
    leftLine2_ts += analogRead(kLineLeftLines[2]);
    rightLine0_ts += analogRead(kLineRightLines[0]);
    rightLine1_ts += analogRead(kLineRightLines[1]);
    rightLine2_ts += analogRead(kLineRightLines[2]);
  }
  //Averaging
  frontLine0_ts /= n;
  frontLine1_ts /= n;
  frontLine2_ts /= n;
  backLine0_ts /= n;
  backLine1_ts /= n;
  backLine2_ts /= n;
  leftLine0_ts /= n;
  leftLine1_ts /= n;
  leftLine2_ts /= n;
  rightLine0_ts /= n;
  rightLine1_ts /= n;
  rightLine2_ts /= n;

  //print all values
  Serial.print("Front line 0ts: ");
  Serial.print(frontLine0_ts);
  Serial.print(" Front line 1ts: ");
  Serial.print(frontLine1_ts);
  Serial.print(" Front line 2ts: ");
  Serial.println(frontLine2_ts);
}

bool frontWhite(){
  uint16_t frontLine0 = analogRead(kLineFrontLines[0]);
  uint16_t frontLine1 = analogRead(kLineFrontLines[1]);
  uint16_t frontLine2 = analogRead(kLineFrontLines[2]);

  if (logLines){
    Serial.print("Front line 0: ");
    Serial.print(frontLine0);
    Serial.print(" Front line 1: ");
    Serial.print(frontLine1);
    Serial.print(" Front line 2: ");
    Serial.println(frontLine2);
    delay(100);
  }
  //reads for white returned >900 for all and 350, 475 and 580 for black
  if (frontLine0 > (frontLine0_ts-linets) && frontLine1 > (frontLine1_ts-linets) && frontLine2 > (frontLine2_ts-linets)){
    return true;
  }

  return false;

}

bool backWhite(){
  uint16_t backLine0 = analogRead(kLineBackLines[0]);
  uint16_t backLine1 = analogRead(kLineBackLines[1]);
  uint16_t backLine2 = analogRead(kLineBackLines[2]);
  
  
  //reads for white returned 700, 900 AND 750 for all and 250, 500 and 150 for black
  if (backLine0 > (backLine0_ts-linets) && backLine1 > (backLine1_ts-linets) && backLine2 > (backLine2_ts-linets)){
    return true;
  }

  return false;

}

bool leftWhite(){
  uint16_t leftLine0 = analogRead(kLineLeftLines[0]);
  uint16_t leftLine1 = analogRead(kLineLeftLines[1]);
  uint16_t leftLine2 = analogRead(kLineLeftLines[2]);
  
  if (logLLines){
    Serial.print("Left line 0: ");
    Serial.print(leftLine0);
    Serial.print(" Left line 1: ");
    Serial.print(leftLine1);
    Serial.print(" Left line 2: ");
    Serial.println(leftLine2);
  }
  
  //reads for white returned 900, 900 AND 900 for all and 250, 450 and 250 for black
  if (leftLine0 > (leftLine0_ts-linets) && leftLine1 > (leftLine1_ts-linets) && leftLine2 > (leftLine2_ts-linets)){
    return true;
  }

  return false;

}

bool rightWhite(){
  uint16_t rightLine0 = analogRead(kLineRightLines[0]);
  uint16_t rightLine1 = analogRead(kLineRightLines[1]);
  uint16_t rightLine2 = analogRead(kLineRightLines[2]);
  
  if (logRLines){
    Serial.print("Right line 0: ");
    Serial.print(rightLine0);
    Serial.print(" Right line 1: ");
    Serial.print(rightLine1);
    Serial.print(" Right line 2: ");
    Serial.println(rightLine2);
  }

  //reads for white returned 800, 900 AND 900 for all and 700, 387 and 320 for black
  if (rightLine0 > (rightLine0_ts-linets) && rightLine1 > (rightLine1_ts-linets) && rightLine2 > (rightLine2_ts-linets)){
    return true;
  }

  return false;

}