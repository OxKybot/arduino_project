
#include <OxKybot_MOTOR_command.h>
#include <OxKybot_JOYSTICK_command.h>
#include<Logger.h>
#include<OxKybot_ARM_command.h>
OxKybot_MOTOR_command motors;
OxKybot_ARM_command arm;
OxKybot_JOYSTICK_command joystick;
Logger logger;

ros::NodeHandle  nh;

/**********************ROS PUBLISHERS**********************/

std_msgs::String str_LXL_msg;
ros::Publisher LXL_node("LXL_node", &str_LXL_msg);

std_msgs::String str_LXR_msg;
ros::Publisher LXR_node("LXR_node", &str_LXR_msg);

std_msgs::String str_ANGLE_msg;
ros::Publisher ANGLE_node("ANGLE_node", &str_ANGLE_msg);

std_msgs::String str_ARDUINO_LOG_msg;
ros::Publisher ARDUINO_LOG_node("ARDUINO_LOG_node", &str_ARDUINO_LOG_msg);

std_msgs::String str_POSITION_msg;
ros::Publisher POSITION_node("POSITION_node", &str_POSITION_msg);
void publishLPOSITION()
{
  addStringToMsg(str_POSITION_msg,"req");
  POSITION_node.publish( &str_POSITION_msg);
}
void publishLARDUINO_LOG(String textLog)
{
  addStringToMsg(str_ARDUINO_LOG_msg,textLog);
  ARDUINO_LOG_node.publish( &str_ARDUINO_LOG_msg);
}
void  addStringToMsg(std_msgs::String str,String input)
{
  int logLength = input.length();
  char *bufferLog = new char[logLength];
  input.toCharArray(bufferLog, logLength);
  str.data = bufferLog;
}
void publishLXL()
{
  uint16_t numL = arm.read_left_arm_position();
  char *LXL_buff = new char[10];
  itoa(numL, LXL_buff, 10);
  str_LXL_msg.data = LXL_buff;
  LXL_node.publish( &str_LXL_msg);
}
void publishLXR()
{

  uint16_t numR = arm.read_right_arm_position();
  char *LXR_buff = new char[10];
  itoa(numR, LXR_buff, 10);
  str_LXR_msg.data = LXR_buff;
  LXR_node.publish( &str_LXR_msg);
}

void publishAngle()
{
  unsigned int angle = motors.getAngle();
  char *ANGLE_buff = new char[10];
  itoa(angle, ANGLE_buff, 10);
  str_ANGLE_msg.data = ANGLE_buff;
  ANGLE_node.publish( &str_ANGLE_msg);
}
/**********************ROS SUBSCRIBERS**********************/
void getPositionResponse( const std_msgs::String& slamPose) {
  motors.setPosition(slamPose);
}
ros::Subscriber<std_msgs::String> subGetPositionCommand("get_position_response", getPositionResponse );
/****************JOYSTICK****************/

void joyCommandMotorCb( const sensor_msgs::Joy& joy) {
  joystick.motor_from_Joymsg(joy);
  publishLPOSITION();
}
void joyCommandArmLeftCb( const sensor_msgs::Joy& joy) {
  joystick.axe_from_Joymsg(joy, true);
  publishLXL();
}
void joyCommandArmRigthCb( const sensor_msgs::Joy& joy) {
  joystick.axe_from_Joymsg(joy, false);
  publishLXR();
}
void joyGotoAngleCommand( const sensor_msgs::Joy& joy) {
  publishAngle();
  joystick.angle_from_Joymsg(joy);
}
ros::Subscriber<sensor_msgs::Joy> subJoyCommandMotor("JoyCommandMotor", joyCommandMotorCb );
ros::Subscriber<sensor_msgs::Joy> subJoyCommandArmLeft("JoyCommandArmLeft", joyCommandArmLeftCb );
ros::Subscriber<sensor_msgs::Joy> subJoyCommandArmRigth("JoyCommandArmRigth", joyCommandArmRigthCb );
ros::Subscriber<sensor_msgs::Joy> subJoyGotoAngleCommand("JoyGotoAngleCommand", joyGotoAngleCommand );

/****************PROG_GOTOANGLE****************/

void gotoAngleCommand(const std_msgs::String& msg) {
  int angle = atoi(msg.data);
  motors.gotoAngle(angle);
}
void resetAngleCommand(const std_msgs::String& msg) {
  motors.resetAngle();
}
ros::Subscriber<std_msgs::String> subGotoAngleCommand("GotoAngleCommand", gotoAngleCommand );
ros::Subscriber<std_msgs::String> subResetAngleCommand("ResetAngleCommand", resetAngleCommand );

/****************PROG_GOTOPOSITION****************/
void goToPositionX(const std_msgs::String& msg) {
  String msgVal = String(msg.data);
  if(msgVal.equals("Go FORWARD"))
  {
    motors.forward(200,0);
  }
  if(msgVal.equals("Go BACWARD"))
  {
    motors.backward(200,0);
  }
  if(msgVal.equals("STOP"))
  {
    motors.motorBrake();
  }
}
ros::Subscriber<std_msgs::String> subGotoPositionCommand("GoToPositionX", goToPositionX );

/****************PROG_MOVE_ARM****************/
void moveLeftArm(const std_msgs::Int16& msg) {
  arm.left_arm_goto_position((uint16_t) msg.data);
}
ros::Subscriber<std_msgs::Int16> subMoveLeftArmCommand("MoveLeftArm", moveLeftArm );
void moveRightArm(const std_msgs::Int16& msg) {
  arm.right_arm_goto_position((uint16_t) msg.data);
}
ros::Subscriber<std_msgs::Int16> subMoveRightArmCommand("MoverightArm", moveRightArm );

/****************************************************/

/**********************ROS SERVICES**********************/
//ros::ServiceClient<std_msgs::String, std_msgs::String> pose_client("pose_container_service");

/****************************************************/
void breakAll()
{
  motors.motorBrake();
}
TimeoutCallback securityTimerMotor = TimeoutCallback(STEP_DURATION,breakAll);

void setup() {
  Serial.begin(57600);
  wdt_enable(WDTO_2S);
  
  //Ros nodes
  nh.initNode();
  //  nh.serviceClient(pose_client);
  nh.subscribe(subJoyCommandMotor);
  nh.subscribe(subJoyCommandArmLeft);
  nh.subscribe(subJoyCommandArmRigth);
  nh.subscribe(subGetPositionCommand);
  nh.subscribe(subGotoAngleCommand);
  nh.subscribe(subResetAngleCommand);
  nh.subscribe(subJoyGotoAngleCommand);
  nh.subscribe(subGotoPositionCommand);
  nh.subscribe(subMoveLeftArmCommand);
  nh.subscribe(subMoveRightArmCommand);

  
  nh.advertise(ANGLE_node);
  nh.advertise(LXL_node);
  nh.advertise(LXR_node);
  nh.advertise(ARDUINO_LOG_node);
  nh.advertise(POSITION_node);
  logger = Logger(&ARDUINO_LOG_node, str_ARDUINO_LOG_msg);
  motors = OxKybot_MOTOR_command();
  motors.setLogger(logger);
  motors.init();
  motors.setTimer(&securityTimerMotor);
  arm = OxKybot_ARM_command();
  arm.setLogger(logger);
  arm.init();
  joystick = OxKybot_JOYSTICK_command();
  joystick.set_motors(motors);
  joystick.set_arm(arm);

}

void loop()
{
  wdt_reset();
  nh.spinOnce();
  securityTimerMotor.loop();
}