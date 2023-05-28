// This class has all the functions related to one Motor.
#ifndef Motor_h
#define Motor_h

#include <Arduino.h>

class Motor {
  public:
    // Motor Characteristics.
    const double pulsesPerRevolution = 300.0;
    
    //////////////////////////////////Constructor//////////////////////////////////////
    Motor();
    Motor(const uint8_t id, const uint8_t digital_one, const uint8_t digital_two, 
    const uint8_t analog_one, const uint8_t encoder_one, const uint8_t encoder_two);
    
    //////////////////////////////////Initialization//////////////////////////////////////
    // init outputs and encoders
    void init();
    
    //////////////////////////////////Motor//////////////////////////////////////
    // Change motor state to forward.
    void forward();
    
    // Change motor state to backward.
    void backward();
    
    // Change motor state to stop.
    void stop();

    // Change motor state to brake.
    void brake();

    // Encoder callback
    void updatePIDTicks();

    void printRPM();

    void changeDirection(uint8_t direction);

    // Change Pwm value of a motor.
    void changePwm(const uint8_t pwm);
    
    // Compute Pid controller and update pwm. 
    void constantRPM(const double velocity);

    void computePID(double target_speed, double &output);

    void resetPID();

    // Set PID Time sample and constants
    void setPID(double kP, double kI, double kD, uint16_t pid_time_sample);

    void setLogPID(bool logPID){
      logPID_ = logPID;
    }
  private:
    // Pins.
    uint8_t id_;
    uint8_t digital_one_;
    uint8_t digital_two_;
    uint8_t pwm_pin_;
    uint8_t encoder_one_;
    uint8_t encoder_two_;

    // Velocity.
    uint8_t pwm_ = 0;
    int encoders_dir_ = 0;
    int pid_ticks_ = 0;
    double current_speed_ = 0;
    double target_speed_ = 0;

    uint8_t minOutput = 0;
    uint8_t maxOutput = 255;
    const int16_t maxErrorSum = 3;
    uint16_t pid_time_sample = 100;
    double kP = 45;
    double kI = 55;
    double kD = 40;
    double error_sum = 0;
    double error_prev = 0;
    
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
};


#endif