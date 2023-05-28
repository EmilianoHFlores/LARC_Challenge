# LARC_Challenge
Arduino sketch for Roborregos' LARC 2023 control challenge. 
Contains two libraries, Motor.h and LARC_Base.h.
Motor.h allows for control of one motor with encoders. It includes a PID function to move at constant RPM. 
LARC_Base.h controls the 4 motors with mecanum and at PID regulated speeds. It includes a second PID compute to use a BNO055 for orientation, with function reorientate() to face a specific angle and functions to move forward and backward with angle corrections.
