#include "DXL.h"

//unomment out if you want debugging messages
//#define DEBUG

dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

int dxl_comm_result   = COMM_TX_FAIL;
int dxlRoll_comm_result  = COMM_TX_FAIL;
int dxlPitch_comm_result = COMM_TX_FAIL;
int dxlYaw_comm_result   = COMM_TX_FAIL;
bool dxl_addparam_result = false;                // addParam result
bool dxl_getdata_result = false;                 // GetParam result
uint8_t dxl_error  = 0;
uint8_t dxlRoll_error  = 0;
uint8_t dxlPitch_error = 0;
uint8_t dxlYaw_error   = 0;


int DXLinit(){

  // Open port
  if (portHandler->openPort())
  {
    #ifdef DEBUG
    //Serial.print("Succeeded to open the port!\n");
    #endif
  }
  else
  {
    #ifdef DEBUG
    //Serial.print("Failed to open the port!\n");
    //Serial.print("Press any key to terminate...\n");
    #endif
    return -1;
  }

  //set baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    #ifdef DEBUG
    //Serial.print("Succeeded to change the baudrate!\n");
    #endif
  }
  else
  {
    #ifdef DEBUG
    //Serial.print("Failed to change the baudrate!\n");
    //Serial.print("Press any key to terminate...\n");
    #endif
    return -1;
  }

  int enableResult = DXLenableAll();
  #ifdef DEBUG
  if(enableResult == 1)
    //Serial.print("Dynamixels has been successfully connected \n");
  #endif

  int homeResult = DXLHomeAll();
  return (enableResult & homeResult);
}

int DXLenable(uint8_t DXL_ID){
  getHandlers();
  flushSerial();
  int dxl_comm_result  = packetHandler->write1ByteTxRx(portHandler, DXL_ID , ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if( dxl_comm_result != COMM_SUCCESS )
    {
    #ifdef DEBUG
    packetHandler->printTxRxResult(dxl_comm_result);
    #endif
    return -1;
  }
  else if (dxl_error != 0)
  {
    #ifdef DEBUG
    packetHandler->printRxPacketError(dxl_error);
    #endif
    return -1;
  }
  else
  {
    #ifdef DEBUG
    char buff[30];
    sprintf(buff,"DXL %d enabled \n", DXL_ID);
    //Serial.print(buff);
    #endif
  }
  return 1;
}

int DXLenableAll(){
    return DXLenable(ROLL_ID)&&DXLenable(PITCH_ID)&&DXLenable(YAW_ID);
}

int DXLdisable(uint8_t DXL_ID){
  getHandlers();
  //flushSerial();
  int dxl_comm_result  = packetHandler->write1ByteTxRx(portHandler, DXL_ID , ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if( dxl_comm_result != COMM_SUCCESS )
    {
    #ifdef DEBUG
    packetHandler->printTxRxResult(dxl_comm_result);
    #endif
    return -1;
  }
  else if (dxl_error != 0)
  {
    #ifdef DEBUG
    packetHandler->printRxPacketError(dxl_error);
    #endif
    return -1;
  }
  else
  {
    #ifdef DEBUG
    char buff[30];
    sprintf(buff,"DXL %d disabled \n", DXL_ID);
    //Serial.print(buff);
    #endif
  }
  return 1;
}

int DXLdisableAll(){
  return DXLdisable(ROLL_ID)&&DXLdisable(PITCH_ID)&&DXLdisable(YAW_ID);
}

int DXLisMoving(uint8_t DXL_ID){
  getHandlers();
  flushSerial();
  uint8_t is_moving;
  dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_MOVING, &is_moving, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
  #ifdef DEBUG
    packetHandler->printTxRxResult(dxl_comm_result);
    #endif
    return -1;
    }
    else if (dxl_error != 0)
    {
    #ifdef DEBUG
    packetHandler->printRxPacketError(dxl_error);
    #endif
    return -1;
  }

  return is_moving;
}

int DXLHomeAll(){
  //Set all motors to low speed while homing
  DXLsetSpeed(ROLL_ID, DXL_HOME_SPEED); DXLsetSpeed(PITCH_ID, DXL_HOME_SPEED); DXLsetSpeed(YAW_ID, DXL_HOME_SPEED);
  delay(100); //wait for transmission

  int result = DXLsetPos(ROLL_ID, ROLL_HOME_POS)&&DXLsetPos(PITCH_ID, PITCH_HOME_POS)&&DXLsetPos(YAW_ID, YAW_HOME_POS);
  while(DXLisMoving(ROLL_ID)||DXLisMoving(PITCH_ID)||DXLisMoving(YAW_ID));
  return result;
}

int DXLsetPos(uint8_t DXL_ID, double rad){
    getHandlers();
    //flushSerial();

    int pos = RAD2COUNTS(rad);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_POSITION, pos, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      #ifdef DEBUG
      packetHandler->printTxRxResult(dxl_comm_result);
      #endif
      return 0;
    }
    else if (dxl_error != 0)
    {
      #ifdef DEBUG
      packetHandler->printRxPacketError(dxl_error);
      #endif
      return 0;
    }
    return 1;
}

int DXLsetSpeed(uint8_t DXL_ID, double rads){
    if(rads > MAX_RADS){
      #ifdef DEBUG
      //Serial.println("RPM out of range");
      #endif
      return 0;
    }

    getHandlers();
    //flushSerial();
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_MOVING_SPEED, RAD2COUNTS(rads), &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      #ifdef DEBUG
      packetHandler->printTxRxResult(dxl_comm_result);
      #endif
      return 0;
    }
    else if (dxl_error != 0)
    {
      #ifdef DEBUG
      packetHandler->printRxPacketError(dxl_error);
      #endif
      return 0;
    }
    return 1;
}

double DXLgetPos(uint8_t DXL_ID){
  getHandlers();
  //flushSerial();
  int32_t present_pos = 0;
  static int32_t previous_pos[DXL_COUNT] = {0};
  dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&present_pos, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS)
  {
    //If you get a failed packet, return last known position
    present_pos = previous_pos[DXL_ID - DXL_MIN];
    #ifdef DEBUG
    //packetHandler->printTxRxResult(dxl_comm_result);
    #endif
    //return -1;
  }
  else if (dxl_error != 0)
  {
    #ifdef DEBUG
    //packetHandler->printRxPacketError(dxl_error);
    #endif
    //return -1;
  }

  double pos = COUNTS2RAD(present_pos);
  //double pos = present_pos;
  previous_pos[DXL_ID - DXL_MIN] = present_pos;
  return pos;
}
double DXLgetSpeed(uint8_t DXL_ID){
  getHandlers();
  //flushSerial();
  uint32_t present_speed;
  dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_SPEED, (uint32_t*)&present_speed, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
  #ifdef DEBUG
    packetHandler->printTxRxResult(dxl_comm_result);
    #endif
    //return -1;
    }
    else if (dxl_error != 0)
    {
    #ifdef DEBUG
    packetHandler->printRxPacketError(dxl_error);
    #endif
    //return -1;
  }

  if(present_speed >= 1 && present_speed <= 1023){
    return COUNTS2RADS(present_speed);
  }else if(present_speed >= 1025 && present_speed <= 2047){
    return COUNTS2RADS(-(present_speed-1023));
  }

  return 0;
}

int DXLsetStates(double (&states)[6]){
  int speedOK, posOK;
  speedOK = posOK = 0;

  // Don't think we need to do this every time, right?
  // speedOK = DXLsetSpeed(ROLL_ID, states[3]) &&
  //               DXLsetSpeed(PITCH_ID, states[4]) &&
  //               DXLsetSpeed(YAW_ID, states[5]);
  speedOK = 1;
  posOK = DXLsetPos(ROLL_ID, states[0]) &&
              DXLsetPos(PITCH_ID, states[1]) &&
              DXLsetPos(YAW_ID, states[2]);

  return speedOK&&posOK;
}

int DXLgetStates(double (&states)[6]){
  states[0] = DXLgetPos(ROLL_ID);
  states[1] = DXLgetPos(PITCH_ID);
  states[2] = DXLgetPos(YAW_ID);

  states[3] = DXLgetSpeed(ROLL_ID);
  states[4] = DXLgetSpeed(PITCH_ID);
  states[5] = DXLgetSpeed(YAW_ID);

  return 1;
}

void normalise_pos(double *rad){
    double rad_n = COUNTS2RAD(COUNTS_PER_REV/2);
    *rad += rad_n;
}
