// namespace std {
//   void __throw_length_error(char const*) {
//    exit(1);
//   }
// }

#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/JointState.h>

#include "motors.h"
#include "motor_params.h"
#include "DXL.h"

#define x_idx      0
#define y_idx      1
#define z_idx      2
#define yaw_idx    3
#define roll_idx   4
#define pitch_idx  5
#define JOINTSTATE_LEN 6

#define ESTOP_PIN   4
#define PUBLISHRATE 10 // in ms
#define STATUSRATE  1000



elapsedMillis sincePublish;
elapsedMillis sinceStatusPublish;


double cart_commands[6] = {0,0,0,0,0,0};
double DXL_commands[6] = {0,0,0,0,0,0};

//function prototypes;
void publishStates(void);
void motor_commandCb(const sensor_msgs::JointState& states);
void getStates();
bool checkEstop();
void publishStates();
void publishStatus();


ros::NodeHandle nh;
sensor_msgs::JointState motor_states;       //name of published topic
sensor_msgs::JointState motor_commands;     //name of subscribed topic
std_msgs::String status_msg;                //name of cartesian status publisher
ros::Publisher motor_states_pub("joint_feedback", &motor_states); //ROS publisher for joint feedback
ros::Publisher robot_status("cartesian_status", &status_msg);     //ROS publisher for robot status



//Callback to revice joint states from PC via ROS message
void motor_commandCb(const sensor_msgs::JointState& states){
  //exectute motor commands

  //joint order: x y z yaw roll pitch
  cart_commands[0] = states.position[x_idx]*1000.0;
  cart_commands[1] = states.position[y_idx]*1000.0;
  cart_commands[2] = states.position[z_idx]*1000.0;

  cart_commands[3] = 1000;     //states.velocity[x_idx];
  cart_commands[4] = 1000;     //states.velocity[y_idx];
  cart_commands[5] = 1000;     //states.velocity[z_idx];

  DXL_commands[0] = states.position[roll_idx];
  DXL_commands[1] = states.position[pitch_idx];
  DXL_commands[2] = states.position[yaw_idx];

  //(DXL_commands[n] > 0) - ( DXL_commands[n] < 0) extracts sign from the desired position
  // + = CCW
  // - = CW
  DXL_commands[3] =  3.14;     //states.velocity[roll_idx];
  DXL_commands[4] =  3.14;     //states.velocity[pitch_idx];
  DXL_commands[5] =  3.14;     //states.velocity[yaw_idx];

  DXLsetStates(DXL_commands);
  setCartState(cart_commands, motor_states.position[x_idx], motor_states.position[y_idx]);
}

ros::Subscriber<sensor_msgs::JointState> motor_commands_sub("motor_command", &motor_commandCb);

void getStates(){
  //Cartesian output in m/s
  motor_states.position[x_idx] = getAxisPosition(XAXIS)/1000.0;
  motor_states.position[y_idx] = getAxisPosition(YAXIS)/1000.0;
  motor_states.position[z_idx] = getAxisPosition(ZAXIS)/1000.0;
  motor_states.position[roll_idx]   = DXLgetPos(ROLL_ID);
  motor_states.position[pitch_idx]  = DXLgetPos(PITCH_ID);
  motor_states.position[yaw_idx]    = DXLgetPos(YAW_ID);

  //motor_states.velocity[x_idx] = getAxisSpeed(XAXIS)/1000.0;
  //motor_states.velocity[y_idx] = getAxisSpeed(YAXIS)/1000.0;
  //motor_states.velocity[z_idx] = getAxisSpeed(ZAXIS)/1000.0;
  // motor_states.velocity[roll_idx] = DXLgetSpeed(ROLL_ID);
  //motor_states.velocity[pitch_idx] = DXLgetSpeed(PITCH_ID);
  // motor_states.velocity[yaw_idx] = DXLgetSpeed(YAW_ID);

  motor_states.velocity[x_idx] = 0;
  motor_states.velocity[y_idx] = 0;
  motor_states.velocity[z_idx] = 0;
  motor_states.velocity[roll_idx]   = 0;
  motor_states.velocity[pitch_idx]  = 0;
  motor_states.velocity[yaw_idx]    = 0;

}

void setup() {
//Init ROS stuff
  pinMode(ESTOP_PIN, INPUT);

  nh.initNode();
  nh.advertise(motor_states_pub);
  nh.advertise(robot_status);
  nh.subscribe(motor_commands_sub);

  while(digitalRead(ESTOP_PIN) != LOW);

  init_motors();
  homeAxis(XAXIS);
  homeAxis(ZAXIS);

  delay(1000);
  DXLinit();
  delay(1000);


  //Init ROS sensor_msg/JointState class memebers
  //motor_states.name_length      = JOINTSTATE_LEN;
  motor_states.velocity_length  = JOINTSTATE_LEN;
  motor_states.position_length  = JOINTSTATE_LEN;
  motor_states.position = new float[JOINTSTATE_LEN];
  motor_states.velocity = new float[JOINTSTATE_LEN];
  //motor_states.name = new char*[JOINTSTATE_LEN];

  motor_commands.velocity_length  = JOINTSTATE_LEN;
  motor_commands.position_length  = JOINTSTATE_LEN;
  motor_commands.position = new float[JOINTSTATE_LEN];
  motor_commands.velocity = new float[JOINTSTATE_LEN];


}


bool checkEstop(){
  static bool dxl_enabled = false;
  //NOTE ESTOP IS NORMALLY CLOSED (NC)
  if(!digitalRead(ESTOP_PIN)){
    //diable motors
    DXLdisableAll();
    dxl_enabled = false;
    return false;
  } else if(dxl_enabled == false) {
    // re-enable the dynamixels
    dxl_enabled = true;
    DXLenableAll();
  }
  return true;
}



void loop() {
  if(checkEstop()) runMotors();

  if(sincePublish >= PUBLISHRATE){
    publishStates();
    sincePublish = 0;
  }

  if(sinceStatusPublish >= STATUSRATE){
    publishStatus();
    sinceStatusPublish = 0;
  }


   nh.spinOnce();
}

void publishStates(){
    getStates();
    motor_states.header.stamp = nh.now();
    motor_states_pub.publish(&motor_states);
}

void publishStatus(){
  //String CP1_status = "CP1 : ";
//  String CP2_status   = "CP2 : ";
//  String CP3_status   = "CP3 : ";
//  String DXL1_status  = "DXL1: ";
//  String DXL2_status  = "DXL1: ";
//  String DXL3_status  = "DXL1: ";
//
//  char cp2_msg[12];
//  char cp3_msg[12];
//
//  getCartStatus(cp2_msg, cp3_msg);
//  CP2_status += String(cp2_msg);
//  CP3_status += String(cp3_msg);
//
//  String status_str = "\n";
//  status_str += CP2_status + '\n' + CP3_status + '\n';
//  status_msg.data = status_str.c_str();
  status_msg.data = "Not implemented yet";
  robot_status.publish(&status_msg);
}
