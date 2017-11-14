#ifndef _DXL_H_
#define _DXL_H_

#include <DynamixelSDK.h>
#define flushSerial() Serial1.flush()


#define getHandlers() dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION)

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          562                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          596
#define ADDR_PRO_PRESENT_POSITION       611
#define ADDR_PRO_MOVING_SPEED           600
#define ADDR_PRO_PRESENT_SPEED          615
#define ADDR_PRO_MOVING                 610


// CONTROL VALUES
#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// DEFAULT SETTINGS
#define DXL_MIN                          2
#define ROLL_ID                          2
#define PITCH_ID                         3
#define YAW_ID                           4
#define DXL_COUNT                        3
#define BAUDRATE                        3000000
#define DEVICENAME                      "COM10"          // redundant since controlling using teensy

#define ROLL_HOME_POS        0    //units - rads
#define ROLL_MAX_ANGLE       360.0
#define ROLL_MIN_ANGLE       0

#define PITCH_HOME_POS       0     //units - rads
#define PITCH_MAX_ANGLE      360.0
#define PITCH_MIN_ANGLE      0

#define YAW_HOME_POS         0
#define YAW_MAX_ANGLE        360.0
#define YAW_MIN_ANGLE        0

#define DXL_HOME_SPEED      (0.52) // Rads/s

#define DEG2RAD(deg)        (deg*2*M_PI/360.0)
#define RAD2DEG(rad)        (rad*360.0/(2*M_PI))

#define COUNTS_PER_REV      361384
#define MAX_ANGLE           360.0
#define MAX_ANGLE_RAD       DEG2RAD(MAX_ANGLE)
#define COUNTS_PER_DEG      (COUNTS_PER_REV/(MAX_ANGLE))
#define COUNTS_PER_RAD      (COUNTS_PER_REV/DEG2RAD(MAX_ANGLE))

#define MAX_RPM             16.0
#define MAX_RADS            (MAX_RPM*M_PI/30.0)
#define COUNTS_PER_RPM      (COUNTS_PER_REV/MAX_RPM)
#define COUNTS_PER_RADS     (COUNTS_PER_REV/MAX_RADS)

#define RPM2COUNTS(rpm)     (rpm*COUNTS_PER_RPM)
#define RADS2COUNTS(rads)   (rads*COUNTS_PER_RADS)

#define COUNTS2RPM(cnts)    (cnts/COUNTS_PER_RPM)
#define COUNTS2RADS(cnts)   (cnts/COUNTS_PER_RADS)

#define COUNTS2DEG(cnts)    (cnts/COUNTS_PER_DEG)
#define COUNTS2RAD(cnts)    (cnts/COUNTS_PER_RAD)
#define DEG2COUNTS(deg)     (COUNTS_PER_DEG*deg)
#define RAD2COUNTS(rad)     (COUNTS_PER_RAD*rad)

//( 361384 / (360.0 *2*M_PI/360.0 )) * rad

int DXLinit();                  //init all DXLs
int DXLHome(uint8_t DXL_ID);
int DXLHomeAll();
int DXLenable(uint8_t DXL_ID);  //enable single DXL
int DXLenableAll();             //enable all DXLs

int DXLdisable(uint8_t DXL_ID); //disable single DXL
int DXLdisableAll();            //disable all DXLs
int DXLisMoving(uint8_t DXL_ID);

/* DXLsetLimits
 * sets max and min limits at specified units
 * INPUT: ID, minAngle, maxAngle, units
 * OUTPUT: Comm result SUCCESS or FAIL
 */
int DXLsetLimits(uint8_t DXL_ID, int minLimit, int maxLmit, uint8_t units);

/* DXLsetPos DXLsetSpeed
 * sets specified motor to desired angle or speed
 * INPUT: ID, desired angle/speed
 * OUTPUT: Comm result SUCCESSS or FAIL
 */
int DXLsetPos(uint8_t DXL_ID, double rad);
int DXLsetSpeed(uint8_t DXL_ID, double rad_per_sec);

/* DXLsetPos DXLsetSpeed
 * returns angle/speed of specified motor
 * INPUT: ID
 * OUTPUT: pos/speed
 */
double DXLgetPos(uint8_t DXL_ID);
double DXLgetSpeed(uint8_t DXL_ID);

/* DXLsetState
* Sets angles of all DXLs
* INPUT: An array of size 6 of DXL positions and speeds in the following order:
*        [roll, pitch, yaw, rollSpeet, yawSpeed, pitchSpeed]
*        eg states[6]: {rollAngle, pitchAngle, yawAngle, rollSpeed, pitchSpeed, yawSpeed}
*
* OUTPUT: Comm result SUCCESS or FAIL
*/
int DXLsetStates(double (&states)[6]);        //set angle/speed in DEGREES of all DXLs at specified units

/* DxlgetState
 * Gets the current state of DXLs at specified units
 * INPUT: A blank array of size 6 for the results to be placed
 *        eg. deg[6]: [roll, pitch, yaw, rollSpeet, yawSpeed, pitchSpeed]
 * OUTPUT: Comm results SUCCESS or FAIL
 */
int DXLgetStates(double (&states)[6]);
void normalise_pos(double *rad);


#endif
