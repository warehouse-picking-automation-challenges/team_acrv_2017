#ifndef _MOTOR_PARAMS_H
#define _MOTOR_PARAMS_H

#include <math.h>

/* PIN DEFININITIONS */

//ENCODERS
#define ENC1APIN 1
#define ENC1BPIN 2
#define ENC2APIN 3
#define ENC2BPIN 4
#define ENC3APIN 5
#define ENC3BPIN 6

#define ENCRES 4090                      //Counts per rev for encoders
#define ENCDEG (360/ENCRES)              //Resolution in Degrees
#define ENCRAD (2*M_PI/ENCRES)           //Resolution in Rads
#define ENC2DIST(enc) (enc*ENCRAD*pulley_radius)  //Linear Distance of motor

//X axis
#define mot1_step_pin    24
#define mot1_dir_pin     25
#define mot1_enable_pin  26


//YZ axis
#define mot2_step_pin    10
#define mot2_dir_pin     11
#define mot2_enable_pin  12
#define mot2_fb_pin      9

//YZ axis
#define mot3_step_pin    6
#define mot3_dir_pin     7
#define mot3_enable_pin  8
#define mot3_fb_pin      5


#define XAXIS 1
#define YAXIS 2
#define ZAXIS 3

#define MOTOR1 1
#define MOTOR2 2
#define MOTOR3 3

/* Clear path variables  */
//Keep this value the same as the clearpath motor!
#define steps_per_rev (800)


/* Conversions  */
#define steps_per_degrees (steps_per_rev/360.0)
#define degrees_per_step (1.0/steps_per_dregrees)
#define pulley_radius (15.289*24/48) //actually the PCD/2  - Scaling for 24 teeth pulleys instead of 48
                                          // ALSO CHANGE INIT_ACCEL WHEN CHANGING PULLEY
#define pulley_circum (2*pulley_radius*M_PI)

#define steps_per_rad (steps_per_rev/(2*M_PI))
#define rads_per_mm (2*M_PI/pulley_circum)
#define steps_per_mm (steps_per_rad * rads_per_mm)

#define dist2step(a) (a * steps_per_mm)
#define step2dist(a) (a / steps_per_mm)

/*  Max and min parameters  */
#define XMAX 980.0
#define XMIN 0.0

#define YMAX 920.0
#define YMIN 0.0

#define ZMAX 860.0
#define ZMIN 0.0

#define min_mm_per_sec (1.0)
#define max_mm_per_sec (2*1000.0)


//IN terms of RPM???
#define oneRPM (steps_per_rev/60.0)
#define init_speed (steps_per_rev/60.0)
#define init_accel (9*10.0*steps_per_rev)  // This is fast enough to mostly keep up with the generated path
                                            //  and slow enough to not break the robot on a step change.


#endif
