#include "packets.h"


//return true if valid packet is valid
bool processPacket(){

  //packet structure:
  // m[x,y,z,dx,dy,dz,roll,pitch,yaw,rollRPM,pitchRPM,yawRPM]    SET STATES
  // r[]                                                         REQUEST STATES
  // c[command]                                                  EXECUTE COMMAND
  String buffer;
  String number = "";
  int idx = 0;;
  double motorCommands[6];
  double wristCommands[6];


  while(Serial.available()){
    //parse the serial into a string
    buffer = Serial.readStringUntil(']');
    buffer += ']';

  }


//check for correct format
  if(buffer[0] != 'm' || buffer[0] != 'c' || buffer[0] != 'r'){
    if( buffer[1] != '[' || !buffer.endsWith(']') ){
      Serial.println(buffer + " is an invalid packet");
      return false;
    }
  }


  //extract movement packet
  if(buffer[0] == 'm'){
    //remove header and end
    buffer.remove(0,1);
    buffer.remove(0,1);
    buffer.remove(buffer.length()-1);

    for(int i = 0; idx < 6; i++){
     if(buffer[i] != ','){
       number += buffer[i];
     }else{
        motorCommands[idx] = (double)(number.toInt());
        number = "";
        Serial.println(motorCommands[idx]);
        idx++;
      }
    }

//   if(idx != 6){
//     Serial.println("Too many or too few input argumenties. Syntax: m[x,y,z,dx,dy,dz]'\\n'");
//     return false;
//   }

   idx = 0;
   number = "";
   for(size_t i = 6; i < buffer.length(); i++){
     if(buffer[i] != ','){
       number += buffer[i];
     }else{
        wristCommands[idx] = (double)(number.toInt());
        number = "";
        Serial.println(wristCommands[idx]);
        idx++;

      }
    }

//   if(idx != 6){
//     Serial.println("Too many or too few input arguments. Syntax: m[x,y,z,dx,dy,dz]'\\n'");
//     return false;
//   }

   setCartState(motorCommands, 1.0, 1.0);
   DXLsetStates(wristCommands);
  }

  if(buffer[0] == 'r'){
    buffer.remove(0); buffer.remove(1); buffer.remove(buffer.length());

    double currentCartState[6];
    double currentWristState[6];
    getCartState(currentCartState);
    DXLgetStates(currentWristState);

    for(int i = 0; i < 6; i++) {Serial.print(currentCartState[i]); Serial.print(" ");}
    Serial.println("");
    for(int i = 0; i < 6; i++) {Serial.print(currentWristState[i]); Serial.print(" ");}
    Serial.println("");
  }

  Serial.println(buffer);
  return true;
}
