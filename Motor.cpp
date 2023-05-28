#include <math.h>
#include "Motor.h"

//////////////////////////////////Constructor//////////////////////////////////////
Motor::Motor() {}
Motor::Motor(const uint8_t id, const uint8_t digital_one, const uint8_t digital_two, 
const uint8_t pwm_pin, const uint8_t encoder_one, const uint8_t encoder_two){
  id_ = id;
  digital_one_ = digital_one;
  digital_two_ = digital_two;
  pwm_pin_ = pwm_pin;
  encoder_one_ = encoder_one;
  encoder_two_ = encoder_two;
}

//////////////////////////////////Initialization//////////////////////////////////////
void Motor::init() {
    pinMode(digital_one_, OUTPUT);
    pinMode(digital_two_, OUTPUT);
    pinMode(pwm_pin_, OUTPUT);
    pinMode(encoder_one_, INPUT);
    pinMode(encoder_two_, INPUT);
    stop();
}

void Motor::updatePIDTicks(){
    //front_left_dir = (int)(digitalRead(kEncoderPinsFrontLeftMotor[1]) == HIGH ? front_left_dir_sign : front_left_dir_sign * -1);
    pid_ticks_++;
}

//////////////////////////////////Motor State//////////////////////////////////////
void Motor::forward() {
  
  digitalWrite(digital_one_, HIGH);
  digitalWrite(digital_two_, LOW);

  analogWrite(pwm_pin_, pwm_);

  if(current_state_ == FORWARD) {
    return;
  }

  resetPID();
  
  current_state_ = FORWARD;
}

void Motor::backward() {
  
  digitalWrite(digital_one_, LOW);
  digitalWrite(digital_two_, HIGH);
  analogWrite(pwm_pin_, pwm_);
  
  if(current_state_ == BACKWARD) {
    return;
  }
  
  digitalWrite(digital_one_, LOW);
  digitalWrite(digital_two_, HIGH);
  
  resetPID();

  current_state_ = BACKWARD;
}

void Motor::stop() {
  pwm_ = 0;
  digitalWrite(digital_one_, LOW);
  digitalWrite(digital_two_, LOW);
  analogWrite(pwm_pin_, LOW);

  if(current_state_ == STOP) {
    return;
  }

  resetPID();
  
  current_state_ = STOP;
}

void Motor::brake() {
  pwm_ = 0;
  digitalWrite(digital_one_, HIGH);
  digitalWrite(digital_two_, HIGH);
  analogWrite(pwm_pin_, HIGH);

  if(current_state_ == BRAKE) {
    return;
  }
  
  resetPID();
  
  current_state_ = BRAKE;
}

void Motor::printRPM(){
  if (millis() - print_time < pid_time_sample){
    return;
  }
  int timeElapsed = millis() - print_time;
  current_speed_ = (pid_ticks_ / pulsesPerRevolution) * 1000.0 / timeElapsed;
  Serial.print(id_);
  Serial.print(" RPM: ");
  Serial.println(current_speed_ * 60);
  print_time = millis();
  pid_ticks_ = 0;
}

void Motor::changeDirection(uint8_t direction){
  current_state_ = direction;
  changePwm(pwm_);
}

void Motor::changePwm(uint8_t pwm) {
  pwm_ = pwm;
  switch(current_state_) {
    case 1:
      forward();
    break;
    case 2:
      backward();
    break;
    case 0:
      stop();
    break;
    case 3:
      brake();
    break;
  }
}

void Motor::constantRPM(const double rpm) {
  double tmp_pwm = pwm_;
  computePID(rpm / 60, tmp_pwm);
  changePwm(tmp_pwm);
  return;
}

void Motor::computePID(double target_speed, double &output) {
  //Jump if timesamplems is not reached
  if (millis() - time < pid_time_sample){
    return;
  }
  //Recalculate current speed
  current_speed_ = (pid_ticks_ / pulsesPerRevolution) * 1000.0 / (millis() - time);
  //Serial.print("Current Speed: ");
  if (logPID_){
    Serial.print(id_);
    Serial.print(" Current Speed: ");
    Serial.println(current_speed_ * 60);
  }
  //Reset tempticks
  pid_ticks_ = 0;
  //Calculate error
  double error = target_speed - current_speed_;
  //Calculate integral
  output = kP * error + kI * error_sum + kD * (error - error_prev);
/*
  Serial.print("Error: ");
  Serial.print(error);
  Serial.print(" Error Sum: ");
  Serial.print(error_sum);
  Serial.print(" Error - Error Prev: ");
  Serial.println(error - error_prev);
  Serial.print("Output: ");
  Serial.println(output);
  */
  //Serial.println("--------------------");
  //Update error_sum and error_prev
  error_sum += error;
  error_prev = error;
  //Limit error_sum
  if (error_sum > maxErrorSum){
    error_sum = maxErrorSum;
  }
  else if (error_sum < -maxErrorSum){
    error_sum = -maxErrorSum;
  }
  //Limit output
  if (output > maxOutput){
    output = maxOutput;
  }
  else if (output < minOutput){
    output = minOutput;
  }
  //Reset time
  time = millis();
  return;
}

void Motor::resetPID(){
  error_sum = 0;
  error_prev = 0;
  time = millis();
  pid_ticks_ = 0;
}

void Motor::setPID(double kP, double kI, double kD, uint16_t pid_time_sample){
  this->kP = kP;
  this->kI = kI;
  this->kD = kD;
  this->pid_time_sample = pid_time_sample;
}