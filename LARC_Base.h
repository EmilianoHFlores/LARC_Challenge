// This class has all the functions related the LARC 2023 Challenge base movement
#ifndef LARC_Base_h
#define LARC_Base_h

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Motor.h"

class LARC_Base {
  public:
    // Motor Characteristics.
    const double pulsesPerRevolution = 300.0;
    
    //////////////////////////////////Constructor//////////////////////////////////////
    LARC_Base();
    LARC_Base(Motor *front_left_motor, Motor *front_right_motor, Motor *back_left_motor, Motor *back_right_motor, Adafruit_BNO055 *bno);
    
    //////////////////////////////////Initialization//////////////////////////////////////
    // init outputs and encoders
    void init();
    void initBNO();
    void resetBNO();
    
    //////////////////////////////////Motor//////////////////////////////////////
    void setPWM(uint8_t pwm);
    void setPWM(uint8_t pwmFL, uint8_t pwmFR, uint8_t pwmBL, uint8_t pwmBR);
    // Change motor state to forward.
    void changeDirection(uint8_t direction);
    void changeDirection(uint8_t directionFL, uint8_t directionFR, uint8_t directionBL, uint8_t directionBR);
    void mecanumLeft();
    void mecanumRight();
    // Encoder callback
    void updateFLPIDTicks();
    void updateFRPIDTicks();
    void updateBLPIDTicks();
    void updateBRPIDTicks();

    void printRPM();

    // Compute Pid controller and update pwm. 
    void baseConstantRPM(const double velocity);

    void baseConstantRPM(const double velocityFL, const double velocityFR, const double velocityBL, const double velocityBR);

    // Combines encoder RPMs and adds a correction factor from the computedAnglePID function, to face in a fixed angle moving forward
    void goForward(const double velocity);
    void goBackwards(const double velocity);

    // Orientates the robot to an angle, using computeAnglePID
    void reorientate(float angle); 

    void setAnglePID(double kP, double kI, double kD, uint16_t pid_time_sample);

    //transforms an angle received from 0 to 360 to -180 to 180
    float transform360to180(float angle);

    //Computes RPMs to reach a target angle for the robot to face
    void computeAnglePID(double target_angle, double &output);

    //Movement with PID from motors + BNO
    void moveForward(const double velocity);

    // Set PID Time sample and constants
    void setPID(double kP, double kI, double kD, uint16_t pid_time_sample);

    void resetPID();

    void setLogPID(bool logPID);
    void setLogAnglePID(bool logAnglePID);

    void printAngle();

    void print180Angle();

    Motor *front_left_motor_;
    Motor *front_right_motor_;
    Motor *back_left_motor_;
    Motor *back_right_motor_;
    Adafruit_BNO055 *bno_;

  private:
  // devices
    

    uint8_t minOutput = 0;
    uint8_t maxOutput = 255;
    const int16_t maxErrorSum = 3;
    uint16_t pid_time_sample = 100;
    double kP = 45;
    double kI = 55;
    double kD = 40;

    float required_angle = 0;
    float current_angle = 0;
    double prev_bno_output = 0;
    double angle_treshold = 1;
    const double angle_treshold_move = 1;
    const double angle_treshold_reorientate = 3.5;

    double anglekP = 45;
    double movekP = 45;
    double reorientatekP = 45;
    double anglekI = 55;
    double anglekD = 40;
    double angle_error_sum = 0;
    double angle_error_prev = 0;
    double max_error_sum= 2;
    double max_output = 330; // max rpm
    double min_output = 0;
    
    //states
    uint8_t FORWARD = 1;
    uint8_t BACKWARD = 2;
    uint8_t STOP = 0;
    uint8_t BRAKE = 3;
    uint8_t current_state_ = STOP;

    //time
    unsigned long time = 0;
    unsigned long print_time = 0;

    //log
    bool logPID_ = false;
    bool logAnglePID_ = false;
};


#endif