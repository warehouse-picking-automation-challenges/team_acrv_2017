#ifndef MOTORS_H_
#define MOTORS_H_

#include <AccelStepper.h>
#include <MultiStepper.h>
#define ENCODER_USE_INTERRUPTS



//#include <Encoder.h>

#define LIMIT3PIN 34
#define LIMIT2PIN 33


/* some useful macros */
#define D2(y, z) ( (y+z) )
#define D3(y, z) ( (y-z) )

#define YDIST(d2, d3) ((d2+d3)/2.0)
#define ZDIST(d2, d3) ((d2-d3)/2.0)


//Function Prototypes
bool checkLimits(unsigned char axis, long position);
bool inMotion();

int setAxisPosition(uint8_t AXIS, double pos);
int setAxisSpeed(uint8_t AXIS, double vel);

double getAxisPosition(uint8_t AXIS);
double getAxisSpeed(uint8_t AXIS);

int setAxisState(uint8_t AXIS);

void init_motors();
void homeAxis(unsigned char axis);
void setCartState(double motorCommand[], double current_x_pos, double current_y_pos);
void getCartState(double (&state)[6] );
void getCartStatus(char* status2, char* status3);
void runMotors();


#endif
