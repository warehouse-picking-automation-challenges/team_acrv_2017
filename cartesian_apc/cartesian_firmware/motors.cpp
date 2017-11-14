#include "motors.h"
#include "motor_params.h"

//Declare encoders
//Encoder enc1(ENC1APIN, ENC1BPIN);
//Encoder enc2(ENC2APIN, ENC2BPIN);
//Encoder enc3(ENC3APIN, ENC3BPIN);

//Declare motors
AccelStepper mot1(AccelStepper::DRIVER, mot1_step_pin, mot1_dir_pin, true);
AccelStepper mot2(AccelStepper::DRIVER, mot2_step_pin, mot2_dir_pin, true);
AccelStepper mot3(AccelStepper::DRIVER, mot3_step_pin, mot3_dir_pin, true);
MultiStepper YZ;


void init_motors(){
  mot1.setAcceleration(init_accel);
  mot1.setMaxSpeed(init_speed);
  mot1.setEnablePin(mot1_enable_pin);
  mot1.setPinsInverted(false, true, true); //direction, step, enable
  mot1.setMinPulseWidth(5);
  mot1.enableOutputs();


  mot2.setAcceleration(init_accel);
  mot2.setMaxSpeed(init_speed);
  mot2.setPinsInverted(true, true, true);
  mot2.setEnablePin(mot2_enable_pin);
  mot2.setMinPulseWidth(5);
  mot2.enableOutputs();
  pinMode(mot2_fb_pin, INPUT_PULLUP);

  mot3.setAcceleration(init_accel);
  mot3.setMaxSpeed(init_speed);
  mot3.setPinsInverted(true, true, true);
  mot3.setMinPulseWidth(5);
  mot3.setEnablePin(mot3_enable_pin);
  mot3.enableOutputs();
  pinMode(mot3_fb_pin, INPUT_PULLUP);

 // XYZ.addStepper(mot1);
  YZ.addStepper(mot2);
  YZ.addStepper(mot3);


  pinMode(LIMIT3PIN, INPUT);
  pinMode(LIMIT2PIN, INPUT);
}

void homeAxis(unsigned char axis){
  switch(axis){
    case XAXIS:
      //Serial.println("Homing X Axis!");
      mot1.disableOutputs();
      delay(50);
      mot1.enableOutputs();
      //Serial.println("Homing done!");
      break;

    case ZAXIS:

      mot2.setMaxSpeed(800);
      mot3.setMaxSpeed(800);
      mot2.setSpeed(800);
      mot3.setSpeed(-800);

      bool run2 , run3;
      run2 = run3 = false;
      while( digitalRead(LIMIT2PIN) != HIGH || digitalRead(LIMIT3PIN) != HIGH) {
        //mot2.runSpeed();


        if(digitalRead(LIMIT2PIN) == HIGH){
          run2 = true;
        }

        if(digitalRead(LIMIT3PIN) == HIGH){
          mot2.setSpeed(-800);
          run2 = true;
        }


        if(run2 == true) mot2.runSpeed();
        mot3.runSpeed();
      }


      mot2.setCurrentPosition( D2(0,dist2step(860)) );
      mot3.setCurrentPosition( D3(0,dist2step(860)) );

      //double motorCommand[] = {getAxisPosition(XAXIS), getAxisPosition(YAXIS), 200, 150,150,150};
      //setCartState(motorCommand, getAxisPosition(XAXIS),  getAxisPosition(YAXIS));


  }

  //Reset Encoders
//  enc1.write(0);
//  enc2.write(0);
//  enc3.write(0);
}


bool checkLimits(unsigned char axis, double position){
  bool good2Go = true;
    switch(axis){
      case XAXIS:
        if(position > XMAX || position < XMIN){
          Serial.println("X command out of bounds");
          good2Go = false;
        }
        break;

      case YAXIS:
        if(position > YMAX || position < YMIN){
          Serial.println("Y command out of bounds");
          good2Go = false;
        }
        break;

       case ZAXIS:
        if(position > ZMAX || position < ZMIN){
          Serial.println("Z command out of bounds");
          good2Go = false;
        }
        break;
    }

  return good2Go;
}

bool checkPhoxLimit(double pos_x, double pos_y) {
  // MAKE SURE WE DON'T HIT THE PHOTO NEO, HAVE HARD LIMITS.
  if(pos_x > 865 && pos_y > 885) {
    return false;
  }
  return true;
}

void runMotors(){
  mot1.run();
  mot2.run();
  mot3.run();
}

bool inMotion(){
  return (mot1.distanceToGo() != 0) || (mot2.distanceToGo() != 0) || (mot3.distanceToGo() != 0);
}


double getAxisPosition(uint8_t AXIS){
  switch(AXIS){
    case XAXIS:
      //return ENC2DIST(enc1.read());
      return step2dist(mot1.currentPosition());

    case YAXIS:
      //return YDIST(ENC2DIST(enc2.read()), ENC2DIST(enc3.read()));
      return YDIST( step2dist(mot2.currentPosition()), step2dist(mot3.currentPosition()) );

    case ZAXIS:
      //return ZDIST(ENC2DIST(enc2.read()), ENC2DIST(enc3.read()));
      return ZDIST( step2dist(mot2.currentPosition()), step2dist(mot3.currentPosition()) );

    default:
      return -1;
  }
  return -1;
}

double getAxisSpeed(uint8_t AXIS){


    switch(AXIS){
    case XAXIS:
      //return ENC2DIST(enc1.read());
      if(mot1.distanceToGo() == 0) return 0;
      return step2dist(mot1.speed());

    case YAXIS:
      //return YDIST(ENC2DIST(enc2.read()), ENC2DIST(enc3.read()));
      if(mot2.distanceToGo() == 0 && mot3.distanceToGo() == 0) return 0;
      return YDIST( step2dist(mot2.speed()), step2dist(mot3.speed()) );

    case ZAXIS:
      //return ZDIST(ENC2DIST(enc2.read()), ENC2DIST(enc3.read()));
      if(mot2.distanceToGo() == 0 && mot3.distanceToGo() == 0) return 0;
      return ZDIST( step2dist(mot2.speed()), step2dist(mot3.speed()) );

    default:
      return -1;
  }
  return -1;

}

void setCartState(double motorCommand[], double current_x_pos,  double current_y_pos){
  //motorCommand should be in the form [x, y, z, dx, dy, dz]
  //Unit input:[mm mm mm mm/s mm/s mm/s]

  bool good2Go = checkLimits(XAXIS, motorCommand[0]) && checkLimits(YAXIS,  motorCommand[1]) && checkLimits(ZAXIS,  motorCommand[2]);
  good2Go = good2Go && checkPhoxLimit(motorCommand[0], motorCommand[1]);  // Don't hit the photoneo, just check commands
  long YZmove[2];
  float YZspeed[2];

  // Check current states for the photoneo
  // Otherwise the commands can be valid but the robot
  // still transitions through positions which hit the photoneo
  // current is in metres, command in mm... confusing right?
  if(current_x_pos > 0.865 && motorCommand[1] > 885) {
    motorCommand[1] = 885;
  } else if(current_y_pos > 0.885 && motorCommand[0] >= 865) {
    motorCommand[0] = 865;
  }

  if(good2Go){
    YZmove[0] = (long)(steps_per_mm*D2( motorCommand[1], motorCommand[2]) );
    YZmove[1] = (long)(steps_per_mm*D3( motorCommand[1], motorCommand[2]) );

    YZspeed[0] = (float)(steps_per_mm * D2(motorCommand[4], motorCommand[5]));
    YZspeed[1] = (float)(steps_per_mm * D3(motorCommand[4], motorCommand[5]));

    mot1.setMaxSpeed( (float)(motorCommand[3]*steps_per_mm) );
    mot1.moveTo( (long)(dist2step(motorCommand[0])) );

    mot2.setMaxSpeed( YZspeed[0] );
    mot2.moveTo( YZmove[0] );

    mot3.setMaxSpeed( YZspeed[0] );
    mot3.moveTo( YZmove[1] );

    //mot2.setMaxSpeed( (float)(motorCommand[3]*steps_per_mm) );
    //mot2.moveTo( (long)(dist2step(motorCommand[0])) );

  }
}

void getCartState(double (&state)[6] ){
  //[x, y, z, dx, dy, dz]
  state[0] = getAxisPosition(XAXIS)/1000.0;
  state[1] = getAxisPosition(YAXIS)/1000.0;
  state[2] = getAxisPosition(ZAXIS)/1000.0;

  state[3] = getAxisSpeed(XAXIS)/1000.0;
  state[4] = getAxisSpeed(YAXIS)/1000.0;
  state[5] = getAxisSpeed(ZAXIS)/1000.0;
}

void getCartStatus(char* status2, char* status3){
  static int cp2_last_shutdown = 0;
  static int cp3_last_shutdown = 0;

  char cp_shutdown_msg[] = "CP Shutdown";
  char cp_okay_msg[] = "CP Okay";

  if(!digitalRead(mot2_fb_pin)){
    cp2_last_shutdown++;
    memcpy(status2, cp_shutdown_msg, strlen(cp_shutdown_msg)+1);
    //attempt to revive motor
    if(cp2_last_shutdown > 3){
      mot2.disableOutputs();
      delay(50);
      mot2.enableOutputs();
      cp2_last_shutdown = 0;
    }
  }else{
    memcpy(status2, cp_okay_msg, strlen(cp_okay_msg)+1);
  }

  if(!digitalRead(mot3_fb_pin)){
    cp3_last_shutdown++;
    memcpy(status3, cp_shutdown_msg, strlen(cp_shutdown_msg)+1);
    if(cp3_last_shutdown > 3){
      mot3.disableOutputs();
      delay(50);
      mot3.enableOutputs();
      cp3_last_shutdown = 0;
    }
  }else{
    memcpy(status3, cp_okay_msg, strlen(cp_okay_msg)+1);
  }

}
