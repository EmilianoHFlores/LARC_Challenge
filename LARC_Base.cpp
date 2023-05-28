#include <math.h>
#include "LARC_Base.h"

//////////////////////////////////Constructor//////////////////////////////////////
LARC_Base::LARC_Base() {}
LARC_Base::LARC_Base(Motor *front_left_motor, Motor *front_right_motor, Motor *back_left_motor, Motor *back_right_motor, Adafruit_BNO055 *bno)
{
  front_left_motor_ = front_left_motor;
  front_right_motor_ = front_right_motor;
  back_left_motor_ = back_left_motor;
  back_right_motor_ = back_right_motor;
  bno_ = bno;
}

//////////////////////////////////Initialization//////////////////////////////////////
void LARC_Base::init()
{
  // init motors
  front_left_motor_->init();
  front_right_motor_->init();
  back_left_motor_->init();
  back_right_motor_->init();
  // init bno
  bno_->begin();
}

void LARC_Base::initBNO()
{
  if (!(bno_->begin()))
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }
  delay(1000);
  bno_->setExtCrystalUse(true);
}

void LARC_Base::resetBNO(){
  initBNO();
  required_angle = 0;
}

void LARC_Base::updateFLPIDTicks()
{
  // front_left_dir = (int)(digitalRead(kEncoderPinsFrontLeftLARC_Base[1]) == HIGH ? front_left_dir_sign : front_left_dir_sign * -1);
  front_left_motor_->updatePIDTicks();
}

void LARC_Base::updateFRPIDTicks()
{
  // front_right_dir = (int)(digitalRead(kEncoderPinsFrontRightLARC_Base[1]) == HIGH ? front_right_dir_sign : front_right_dir_sign * -1);
  front_right_motor_->updatePIDTicks();
}

void LARC_Base::updateBLPIDTicks()
{
  // back_left_dir = (int)(digitalRead(kEncoderPinsBackLeftLARC_Base[1]) == HIGH ? back_left_dir_sign : back_left_dir_sign * -1);
  back_left_motor_->updatePIDTicks();
}

void LARC_Base::updateBRPIDTicks()
{
  // back_right_dir = (int)(digitalRead(kEncoderPinsBackRightLARC_Base[1]) == HIGH ? back_right_dir_sign : back_right_dir_sign * -1);
  back_right_motor_->updatePIDTicks();
}

//////////////////////////////////LARC_Base State//////////////////////////////////////
void LARC_Base::setPWM(uint8_t pwm)
{
  front_left_motor_->changePwm(pwm);
  front_right_motor_->changePwm(pwm);
  back_left_motor_->changePwm(pwm);
  back_right_motor_->changePwm(pwm);
}

void LARC_Base::setPWM(uint8_t pwmFL, uint8_t pwmFR, uint8_t pwmBL, uint8_t pwmBR)
{
  front_left_motor_->changePwm(pwmFL);
  front_right_motor_->changePwm(pwmFR);
  back_left_motor_->changePwm(pwmBL);
  back_right_motor_->changePwm(pwmBR);
}

void LARC_Base::changeDirection(uint8_t direction)
{
  front_left_motor_->changeDirection(direction);
  front_right_motor_->changeDirection(direction);
  back_left_motor_->changeDirection(direction);
  back_right_motor_->changeDirection(direction);
}

void LARC_Base::changeDirection(uint8_t directionFL, uint8_t directionFR, uint8_t directionBL, uint8_t directionBR)
{
  front_left_motor_->changeDirection(directionFL);
  front_right_motor_->changeDirection(directionFR);
  back_left_motor_->changeDirection(directionBL);
  back_right_motor_->changeDirection(directionBR);
}

void LARC_Base::mecanumLeft()
{
  changeDirection(BACKWARD, FORWARD, FORWARD, BACKWARD);
}

void LARC_Base::mecanumRight()
{
  changeDirection(FORWARD, BACKWARD, BACKWARD, FORWARD);
}

void LARC_Base::printRPM()
{
  front_left_motor_->printRPM();
  front_right_motor_->printRPM();
  back_left_motor_->printRPM();
  back_right_motor_->printRPM();
}

void LARC_Base::baseConstantRPM(const double rpm)
{
  front_left_motor_->constantRPM(rpm);
  front_right_motor_->constantRPM(rpm);
  back_left_motor_->constantRPM(rpm);
  back_right_motor_->constantRPM(rpm);
}

void LARC_Base::baseConstantRPM(const double rpmFL, const double rpmFR, const double rpmBL, const double rpmBR)
{
  front_left_motor_->constantRPM(rpmFL);
  front_right_motor_->constantRPM(rpmFR);
  back_left_motor_->constantRPM(rpmBL);
  back_right_motor_->constantRPM(rpmBR);
}

// uses both PIDs to move forward
void LARC_Base::goForward(const double velocity)
{
  double output = prev_bno_output;
  kP = movekP;
  angle_treshold = angle_treshold_move;
  changeDirection(FORWARD);
  computeAnglePID(required_angle, output);
  output = abs(output);
  if (current_angle < required_angle)
  {
    // turn right
    baseConstantRPM(velocity + output, velocity - output, velocity + output, velocity - output);
  }
  else
  {
    // turn left
    baseConstantRPM(velocity - output, velocity + output, velocity - output, velocity + output);
  }
  prev_bno_output = output;
}

// uses both PIDs to move backwards
void LARC_Base::goBackwards(const double velocity)
{
  double output = prev_bno_output;
  kP = movekP;
  angle_treshold = angle_treshold_move;
  changeDirection(BACKWARD);
  computeAnglePID(required_angle, output);
  output = abs(output);
  if (current_angle > required_angle)
  {
    // turn right
    baseConstantRPM(velocity + output, velocity - output, velocity + output, velocity - output);
  }
  else
  {
    // turn left
    baseConstantRPM(velocity - output, velocity + output, velocity - output, velocity + output);
  }
  prev_bno_output = output;
}

void LARC_Base::reorientate(float angle)
{
  required_angle = angle;
  kP = reorientatekP;
  angle_treshold = angle_treshold_reorientate;
  double output;
  computeAnglePID(angle, output);
  while (output != 0)
  {
    output = abs(output);
    double rightRPM = output;
    double leftRPM = output;
    if (current_angle < angle)
    {
      // turn right
      changeDirection(FORWARD, BACKWARD, FORWARD, BACKWARD);
      baseConstantRPM(leftRPM, rightRPM, leftRPM, rightRPM);
    }
    else
    {
      // turn left
      changeDirection(BACKWARD, FORWARD, BACKWARD, FORWARD);
      baseConstantRPM(leftRPM, rightRPM, leftRPM, rightRPM);
    }
    computeAnglePID(angle, output);
  }
}

void LARC_Base::setAnglePID(double kP, double kI, double kD, uint16_t pid_time_sample)
{
  anglekP = kP;
  movekP = kP;
  reorientatekP = kP-2;
  anglekI = kI;
  anglekD = kD;
  this->pid_time_sample = pid_time_sample;
}

float LARC_Base::transform360to180(float angle)
{
  if (angle > 180)
  {
    angle = -(360 - angle);
  }
  return angle;
}

void LARC_Base::computeAnglePID(double target_angle, double &output)
{
  if (millis() - time < pid_time_sample)
  {
    return;
  }
  sensors_event_t event;
  bno_->getEvent(&event);
  current_angle = transform360to180(event.orientation.x);
  double error = target_angle - current_angle;
  output = anglekP * error + anglekI * angle_error_sum + anglekD * (error - angle_error_prev);
  if (abs(error) < angle_treshold)
  {
    output = 0;
  }
  if (logAnglePID_){
    Serial.print("Current angle: ");
    Serial.print(current_angle);
    Serial.print(" Target angle: ");
    Serial.print(target_angle);
    Serial.print(" Output: ");
    Serial.println(output);
  }
  if (output == 0)
  {
    time = millis();
    return;
  }
  angle_error_sum += error;
  angle_error_prev = error;
  if (angle_error_sum > max_error_sum)
  {
    angle_error_sum = max_error_sum;
  }
  else if (angle_error_sum < -max_error_sum)
  {
    angle_error_sum = -max_error_sum;
  }
  if (output > max_output)
  {
    output = max_output;
  }
  else if (output < -max_output)
  {
    output = -max_output;
  }
  output = abs(output);
  angle_error_sum = abs(angle_error_sum);
  time = millis();
}

void LARC_Base::resetPID()
{
  front_left_motor_->resetPID();
  front_right_motor_->resetPID();
  back_left_motor_->resetPID();
  back_right_motor_->resetPID();
}

void LARC_Base::setPID(double kP, double kI, double kD, uint16_t pid_time_sample)
{
  this->kP = kP;
  this->kI = kI;
  this->kD = kD;
  this->pid_time_sample = pid_time_sample;
}

void LARC_Base::setLogPID(bool logPID)
{
  front_left_motor_->setLogPID(logPID);
  front_right_motor_->setLogPID(logPID);
  back_left_motor_->setLogPID(logPID);
  back_right_motor_->setLogPID(logPID);
}

void LARC_Base::setLogAnglePID(bool logAnglePID)
{
  logAnglePID_ = logAnglePID;
}

void LARC_Base::printAngle()
{
  sensors_event_t event;
  bno_->getEvent(&event);
  Serial.print("X: ");
  Serial.print(event.orientation.x);
  Serial.print(" Y: ");
  Serial.print(event.orientation.y);
  Serial.print(" Z: ");
  Serial.println(event.orientation.z);
}

void LARC_Base::print180Angle()
{
  sensors_event_t event;
  bno_->getEvent(&event);
  Serial.print("X: ");
  Serial.print(transform360to180(event.orientation.x));
  Serial.print(" Y: ");
  Serial.print(transform360to180(event.orientation.y));
  Serial.print(" Z: ");
  Serial.println(transform360to180(event.orientation.z));
}