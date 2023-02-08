#define ROSSERIAL_ARDUINO_TCP
#define sample_num_mdate  5000

#include <SPI.h>
#include <ros.h>
#include <Ethernet.h>
#include <geometry_msgs/Twist.h>
#include <Arduino_PortentaBreakout.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h> //bachelor change
#include <VL53L1X.h>
#include <Wire.h>
#include <EthernetUdp.h>
#include "MPU9250.h"
#include "I2Cdev.h"

breakoutPin ECA1 = GPIO_3;
breakoutPin ECB1 = GPIO_4;
breakoutPin IN1 = PWM2;
breakoutPin IN2 = PWM3;
breakoutPin ENA = PWM0;

breakoutPin ECA2 = GPIO_5;
breakoutPin ECB2 = GPIO_6;
breakoutPin IN3 = PWM4;
breakoutPin IN4 = PWM5;
breakoutPin ENB = PWM1;

breakoutPin xshut0 = PWM6; breakoutPin xshut1 = GPIO_1; breakoutPin xshut2 = GPIO_2; breakoutPin xshut3 = GPIO_0;

breakoutPin xshut4 = ANALOG_A4; breakoutPin xshut5 = PWM7; breakoutPin xshut6 = PWM8; breakoutPin xshut7 = PWM9;

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(172, 17, 10, 173);
IPAddress server(172, 17, 10, 3);

VL53L1X pololu0;
VL53L1X pololu1;
VL53L1X pololu2;
VL53L1X pololu3;
VL53L1X pololu4;
VL53L1X pololu5;
VL53L1X pololu6;
VL53L1X pololu7;

std_msgs::Float64 sensor0;
std_msgs::Float64 sensor1;
std_msgs::Float64 sensor2;
std_msgs::Float64 sensor3;
std_msgs::Float64 sensor4;
std_msgs::Float64 sensor5;
std_msgs::Float64 sensor6;
std_msgs::Float64 sensor7;


const uint16_t serverPort = 11411;
const uint16_t Port = 5000;

ros::NodeHandle nh;

double pwmLeftReq = 0;
double pwmRightReq = 0;

float left = 0;
float right = 0;

static int pwmRightOut = 0;
static int pwmLeftOut = 0;

boolean Direction_left = true;
boolean Direction_right = true;

const int encoder_minimum = -32768;
const int encoder_maximum = 32767;

std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("Encoder_Left", &right_wheel_tick_count);

std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("Encoder_Right", &left_wheel_tick_count);

enum control_mode
{
  CMD_VEL,
  INDPENDENT_CTRL
};

control_mode ctrl_mode = CMD_VEL;


void change_topic(const std_msgs::String& cmd_msg)
{
  if (cmd_msg.data[0] == 'q')
  {
    ctrl_mode = CMD_VEL;
  }
  if (cmd_msg.data[0] == 'a')
  {
    ctrl_mode = INDPENDENT_CTRL;
  }
}

void controlRight(const std_msgs::Float64& pwm)
{
  if (ctrl_mode == INDPENDENT_CTRL)
  {
    right = pwm.data;
  }
}


void controlLeft(const std_msgs::Float64& pwm)
{
  if (ctrl_mode == INDPENDENT_CTRL)
  {
    left = pwm.data;
  }
}

ros::Publisher Pololu0("Pololu0", &sensor0);
ros::Publisher Pololu1("Pololu1", &sensor1);
ros::Publisher Pololu2("Pololu2", &sensor2);
ros::Publisher Pololu3("Pololu3", &sensor3);
ros::Publisher Pololu4("Pololu4", &sensor4);
ros::Publisher Pololu5("Pololu5", &sensor5);
ros::Publisher Pololu6("Pololu6", &sensor6);
ros::Publisher Pololu7("Pololu7", &sensor7);

ros::Subscriber<std_msgs::String> changeTopic("change_topic", &change_topic);
ros::Subscriber<std_msgs::Float64> rightWheel("control_right", &controlRight);
ros::Subscriber<std_msgs::Float64> leftWheel("control_left", &controlLeft);

const int interval = 30;
long previousMillis = 0;
long currentMillis = 0;

const int PWM_INCREMENT = 1;
const int TICKS_PER_REVOLUTION = 620;
const double WHEEL_RADIUS = 0.06;


const double WHEEL_BASE = 0.32;
const double TICKS_PER_METER = 4500;
const int K_P = 178;
const int b = 52;
const int P = 250;
const int PWM_TURN = 70;
const int PWM_MIN = 50 ;
const int PWM_MAX = 70;

double velLeftWheel = 0;
double velRightWheel = 0;
double lastCmdVelReceived = 0;


void right_wheel_tick()
{
  int val = digitalRead(ECB2);

  if (val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }

  if (Direction_right) {

    if (right_wheel_tick_count.data == encoder_maximum) {
      right_wheel_tick_count.data = encoder_minimum;
    }
    else {
      right_wheel_tick_count.data--;
    }
  }
  else {
    if (right_wheel_tick_count.data == encoder_minimum) {
      right_wheel_tick_count.data = encoder_maximum;
    }
    else {
      right_wheel_tick_count.data++;
    }
  }
}

void left_wheel_tick()
{
  int val = digitalRead(ECB1);

  if (val == LOW) {
    Direction_left = true; // Reverse
  }
  else {
    Direction_left = false; // Forward
  }

  if (Direction_left) {
    if (left_wheel_tick_count.data == encoder_maximum) {
      left_wheel_tick_count.data = encoder_minimum;
    }
    else {
      left_wheel_tick_count.data--;
    }
  }
  else {
    if (left_wheel_tick_count.data == encoder_minimum) {
      left_wheel_tick_count.data = encoder_maximum;
    }
    else {
      left_wheel_tick_count.data++;
    }
  }
}

void calc_vel_left_wheel()
{

  static double prevTime = 0;
  static int prevLeftCount = 0;
  int numOfTicks = (65535 + left_wheel_tick_count.data - prevLeftCount) % 65535;
  if (numOfTicks > 10000)
  {
    numOfTicks = 0 - (65535 - numOfTicks);
  }
  velLeftWheel = numOfTicks / TICKS_PER_METER / ((millis() / 1000) - prevTime);
  prevLeftCount = left_wheel_tick_count.data;
  prevTime = (millis() / 1000);

}

void calc_vel_right_wheel()
{
  static double prevTime = 0;
  static int prevRightCount = 0;
  int numOfTicks = (65535 + right_wheel_tick_count.data - prevRightCount) % 65535;
  if (numOfTicks > 10000)
  {
    numOfTicks = 0 - (65535 - numOfTicks);
  }

  velRightWheel = numOfTicks / TICKS_PER_METER / ((millis() / 1000) - prevTime);
  prevRightCount = right_wheel_tick_count.data;
  prevTime = (millis() / 1000);
}

void calc_pwm_values(const geometry_msgs::Twist& cmdVel)
{
  if (ctrl_mode == CMD_VEL)
  {
    lastCmdVelReceived = (millis() / 1000);

    pwmLeftReq = K_P * cmdVel.linear.x + b;
    pwmRightReq = K_P * cmdVel.linear.x + b;

    if (cmdVel.angular.z != 0.0) {

      if (cmdVel.angular.z < 0.0) {
        pwmLeftReq = PWM_TURN;
        pwmRightReq = -PWM_TURN;
      }
      else {
        pwmLeftReq = -PWM_TURN;
        pwmRightReq = PWM_TURN;
      }
    }
    else {

      static double prevDiff = 0;
      static double prevPrevDiff = 0;
      double currDifference = velLeftWheel - velRightWheel;
      double avgDifference = (prevDiff + prevPrevDiff + currDifference) / 3;
      prevPrevDiff = prevDiff;
      prevDiff = currDifference;

      pwmLeftReq += (int)(avgDifference * P);
      pwmRightReq -= (int)(avgDifference * P);
    }

    if (abs(pwmLeftReq) < PWM_MIN) {
      pwmLeftReq = 0;
    }
    if (abs(pwmRightReq) < PWM_MIN) {
      pwmRightReq = 0;
    }
  }
}

void set_pwm_values() {

  static int pwmRightOut = 0;
  static int pwmLeftOut = 0;
  static bool stopped = false;
  
  if (pwmLeftReq > 0) { // Left wheel forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else if (pwmLeftReq < 0) { // Left wheel reverse
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  else if (pwmLeftReq == 0 && pwmLeftOut == 0 ) { // Left wheel stop
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  else { // Right wheel stop
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  if (pwmRightReq > 0) { // Right wheel forward
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  else if (pwmRightReq < 0) { // Right wheel reverse
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  else if (pwmRightReq == 0 && pwmRightOut == 0) { // Right wheel stop
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
  else { // Left wheel stop
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  if (pwmLeftReq != 0 && velLeftWheel == 0) {
    pwmLeftReq *= 1.5;
  }
  if (pwmRightReq != 0 && velRightWheel == 0) {
    pwmRightReq *= 1.5;
  }

  if (abs(pwmLeftReq) > pwmLeftOut) {
    pwmLeftOut += PWM_INCREMENT;
  }
  else if (abs(pwmLeftReq) < pwmLeftOut) {
    pwmLeftOut -= PWM_INCREMENT;
  }
  else {}

  if (abs(pwmRightReq) > pwmRightOut) {
    pwmRightOut += PWM_INCREMENT;
  }
  else if (abs(pwmRightReq) < pwmRightOut) {
    pwmRightOut -= PWM_INCREMENT;
  }
  else {}

  pwmLeftOut = (pwmLeftOut > PWM_MAX) ? PWM_MAX : pwmLeftOut;
  pwmRightOut = (pwmRightOut > PWM_MAX) ? PWM_MAX : pwmRightOut;

  pwmLeftOut = (pwmLeftOut < 0) ? 0 : pwmLeftOut;
  pwmRightOut = (pwmRightOut < 0) ? 0 : pwmRightOut;

  analogWrite(ENA, pwmRightOut);
  analogWrite(ENB, pwmLeftOut);
}

ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &calc_pwm_values );

void MotorL(int Pulse_Width1) {
  if (Pulse_Width1 > 0) {

    analogWrite(ENA, Pulse_Width1);

    digitalWrite(IN1, LOW);

    digitalWrite(IN2, HIGH);

  }

  if (Pulse_Width1 < 0) {

    Pulse_Width1 = abs(Pulse_Width1);

    analogWrite(ENA, Pulse_Width1);

    digitalWrite(IN1, HIGH);

    digitalWrite(IN2, LOW);

  }

  if (Pulse_Width1 == 0) {

    analogWrite(ENA, Pulse_Width1);

    digitalWrite(IN1, LOW);

    digitalWrite(IN2, LOW);

  }

}


void MotorR(int Pulse_Width2) {


  if (Pulse_Width2 > 0) {

    analogWrite(ENB, Pulse_Width2);

    digitalWrite(IN3, HIGH);

    digitalWrite(IN4, LOW);

  }

  if (Pulse_Width2 < 0) {

    Pulse_Width2 = abs(Pulse_Width2);

    analogWrite(ENB, Pulse_Width2);

    digitalWrite(IN3, LOW);

    digitalWrite(IN4, HIGH);

  }

  if (Pulse_Width2 == 0) {

    analogWrite(ENB, Pulse_Width2);

    digitalWrite(IN3, LOW);

    digitalWrite(IN4, LOW);

  }
}


void setup()
{
  Wire.begin();
  Wire.setClock(400000);


  Serial.begin(9600);
  pinMode(ECA1 , INPUT_PULLUP);
  pinMode(ECB1 , INPUT);
  pinMode(ECA2 , INPUT_PULLUP);
  pinMode(ECB2 , INPUT);

  attachInterrupt(digitalPinToInterrupt(ECA1), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ECA2), right_wheel_tick, RISING);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  Breakout.pinMode(xshut0, OUTPUT); //pololu0 XSHUT
  Breakout.pinMode(xshut1, OUTPUT); //pololu1 XSHUT
  Breakout.pinMode(xshut2, OUTPUT); //pololu2 XSHUT
  Breakout.pinMode(xshut3, OUTPUT); //pololu3 XSHUT
  Breakout.pinMode(xshut4, OUTPUT); //pololu0 XSHUT
  Breakout.pinMode(xshut5, OUTPUT); //pololu1 XSHUT
  Breakout.pinMode(xshut6, OUTPUT); //pololu2 XSHUT
  Breakout.pinMode(xshut7, OUTPUT); //pololu3 XSHUT


  //GPIO pinouts must be on LOW value, otherwise we will damaged the VL53L1X
  Breakout.digitalWrite(xshut0, LOW);
  Breakout.digitalWrite(xshut1, LOW);
  Breakout.digitalWrite(xshut2, LOW);
  Breakout.digitalWrite(xshut3, LOW);
  Breakout.digitalWrite(xshut4, LOW);
  Breakout.digitalWrite(xshut5, LOW);
  Breakout.digitalWrite(xshut6, LOW);
  Breakout.digitalWrite(xshut7, LOW);


  /* pololu0 */
  // allow the sensor board pololu0 to pull its XSHUT pin up to 2.8V
  pinMode(xshut0, INPUT);
  pololu0.init();
  pololu0.setAddress((uint8_t)0x2a); //each pololu has to have its unique address for proper working I2C

  /* pololu1 */
  // allow the sensor board pololu0 to pull its XSHUT pin up to 2.8V
  pinMode(xshut1, INPUT);
  pololu1.init();
  pololu1.setAddress((uint8_t)0x2b); //each pololu has to have its unique address for proper working I2C

  /* pololu2 */
  // allow the sensor board pololu0 to pull its XSHUT pin up to 2.8V
  pinMode(xshut2, INPUT);
  pololu2.init();
  pololu2.setAddress((uint8_t)0x2c); //each pololu has to have its unique address for proper working I2C

  /* pololu3 */
  // allow the sensor board pololu0 to pull its XSHUT pin up to 2.8V
  pinMode(xshut3, INPUT);
  pololu3.init();
  pololu3.setAddress((uint8_t)0x2d); //each pololu has to have its unique address for proper working I2C


  /* pololu4 */
  // allow the sensor board pololu0 to pull its XSHUT pin up to 2.8V
  pinMode(xshut4, INPUT);
  pololu4.init();
  pololu4.setAddress((uint8_t)0x1a); //each pololu has to have its unique address for proper working I2C

  /* pololu5 */
  // allow the sensor board pololu0 to pull its XSHUT pin up to 2.8V
  pinMode(xshut5, INPUT);
  pololu5.init();
  pololu5.setAddress((uint8_t)0x1b); //each pololu has to have its unique address for proper working I2C

  /* pololu6 */
  // allow the sensor board pololu0 to pull its XSHUT pin up to 2.8V
  pinMode(xshut6, INPUT);
  pololu6.init();
  pololu6.setAddress((uint8_t)0x1c); //each pololu has to have its unique address for proper working I2C

  /* pololu7 */
  // allow the sensor board pololu0 to pull its XSHUT pin up to 2.8V
  pinMode(xshut7, INPUT);
  pololu7.init();
  pololu7.setAddress((uint8_t)0x1d); //each pololu has to have its unique address for proper working I2C

  pololu0.setDistanceMode(VL53L1X::Long);
  pololu0.setMeasurementTimingBudget(33000);
  pololu0.startContinuous(33);

  pololu1.setDistanceMode(VL53L1X::Long);
  pololu1.setMeasurementTimingBudget(33000);
  pololu1.startContinuous(33);


  pololu2.setDistanceMode(VL53L1X::Long);
  pololu2.setMeasurementTimingBudget(33000);
  pololu2.startContinuous(33);


  pololu3.setDistanceMode(VL53L1X::Long);
  pololu3.setMeasurementTimingBudget(33000);
  pololu3.startContinuous(33);


  pololu4.setDistanceMode(VL53L1X::Long);
  pololu4.setMeasurementTimingBudget(33000);
  pololu4.startContinuous(33);


  pololu5.setDistanceMode(VL53L1X::Long);
  pololu5.setMeasurementTimingBudget(33000);
  pololu5.startContinuous(33);

  pololu6.setDistanceMode(VL53L1X::Long);
  pololu6.setMeasurementTimingBudget(33000);
  pololu6.startContinuous(33);

  pololu7.setDistanceMode(VL53L1X::Long);
  pololu7.setMeasurementTimingBudget(33000);
  pololu7.startContinuous(33);

  Ethernet.begin(mac, ip);
  nh.getHardware()->setConnection(server, serverPort);

  nh.initNode();
  nh.subscribe(changeTopic);
  nh.subscribe(leftWheel);
  nh.subscribe(rightWheel);
  nh.advertise(rightPub);
  nh.advertise(leftPub);
  nh.subscribe(subCmdVel);
  nh.advertise(Pololu0);
  nh.advertise(Pololu1);
  nh.advertise(Pololu2);
  nh.advertise(Pololu3);
  nh.advertise(Pololu4);
  nh.advertise(Pololu5);
  nh.advertise(Pololu6);
  nh.advertise(Pololu7);

}

void loop()
{
  nh.spinOnce();

  currentMillis = millis();

  sensor0.data = pololu0.read();
  sensor1.data = pololu1.read();
  sensor2.data = pololu2.read();
  sensor3.data = pololu3.read();
  sensor4.data = pololu4.read();
  sensor5.data = pololu5.read();
  sensor6.data = pololu6.read();
  sensor7.data = pololu7.read();

  Pololu0.publish(&sensor0);
  Pololu1.publish(&sensor1);
  Pololu2.publish(&sensor2);
  Pololu3.publish(&sensor3);
  Pololu4.publish(&sensor4);
  Pololu5.publish(&sensor5);
  Pololu6.publish(&sensor6);
  Pololu7.publish(&sensor7);

  if (currentMillis - previousMillis > interval) {

    previousMillis = currentMillis;

    leftPub.publish( &left_wheel_tick_count );
    rightPub.publish( &right_wheel_tick_count );

    calc_vel_right_wheel();
    calc_vel_left_wheel();
  }
  if (ctrl_mode == CMD_VEL) {
    set_pwm_values();
  }
  else if (ctrl_mode == INDPENDENT_CTRL) {
    MotorR(-right);
    MotorL(left);
  }
  if ((millis() / 1000) - lastCmdVelReceived > 1) {
    pwmLeftReq = 0;
    pwmRightReq = 0;
  }
}
