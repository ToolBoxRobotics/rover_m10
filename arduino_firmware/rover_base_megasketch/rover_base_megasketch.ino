#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

// ====== PCA9685 Setup ======
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Servo configuration
#define SERVO_MIN_PULSE  150 // 0 deg (approx)
#define SERVO_MAX_PULSE  600 // 180 deg (approx)

// Wheel indices
enum WheelIndex {
  W_FL = 0,
  W_FM = 1,
  W_FR = 2,
  W_RL = 3,
  W_RM = 4,
  W_RR = 5
};

// Servo indices
enum ServoIndex {
  S_FL = 0,
  S_FR = 1,
  S_RL = 2,
  S_RR = 3
};

// PCA9685 channels for motors and servos
const uint8_t motorPwmChannel[6] = {0, 1, 2, 3, 4, 5};
const uint8_t steeringServoChannel[4] = {8, 9, 10, 11};

// ====== Encoder Setup ======
volatile long wheelTicks[6] = {0, 0, 0, 0, 0, 0};

// Example pin assignments (A channel only shown in ISRs; B used for direction)
const uint8_t encA[6] = {2, 3, 18, 19, 20, 21};
const uint8_t encB[6] = {22, 23, 24, 25, 26, 27};

// ====== ROS Setup ======
ros::NodeHandle nh;

// Incoming commands
std_msgs::Float32MultiArray wheel_speed_cmd_msg;
std_msgs::Float32MultiArray steering_cmd_msg;

// Outgoing encoder ticks
std_msgs::Int32MultiArray wheel_ticks_msg;

// Forward declarations
void wheelSpeedCmdCb(const std_msgs::Float32MultiArray &msg);
void steeringCmdCb(const std_msgs::Float32MultiArray &msg);

ros::Subscriber<std_msgs::Float32MultiArray> sub_wheel_speed("wheel_speed_cmd", &wheelSpeedCmdCb);
ros::Subscriber<std_msgs::Float32MultiArray> sub_steering("steering_cmd", &steeringCmdCb);
ros::Publisher pub_wheel_ticks("wheel_ticks", &wheel_ticks_msg);

// ====== Helper: servo angle to PWM ======
uint16_t angleToPulse(float angle_deg) {
  if (angle_deg < -45) angle_deg = -45;
  if (angle_deg >  45) angle_deg =  45;
  float norm = (angle_deg + 45.0) / 90.0; // 0..1
  return (uint16_t)(SERVO_MIN_PULSE + norm * (SERVO_MAX_PULSE - SERVO_MIN_PULSE));
}

// ====== Helper: motor speed [-1..1] to PWM (0..4095) ======
uint16_t speedToPwm(float speed) {
  if (speed < -1.0) speed = -1.0;
  if (speed >  1.0) speed =  1.0;
  return (uint16_t)(fabs(speed) * 4095.0);
}

// ====== Encoder ISRs (A-channel) ======
void encISR0() {
  bool a = digitalRead(encA[0]);
  bool b = digitalRead(encB[0]);
  wheelTicks[0] += (a == b) ? 1 : -1;
}
void encISR1() {
  bool a = digitalRead(encA[1]);
  bool b = digitalRead(encB[1]);
  wheelTicks[1] += (a == b) ? 1 : -1;
}
// Add ISR2..ISR5 similarly, or use pin change interrupts as needed.

// ====== ROS Callbacks ======
void wheelSpeedCmdCb(const std_msgs::Float32MultiArray &msg) {
  // Expect 6 elements: normalized speeds [-1..1]
  if (msg.data_length < 6) return;
  for (int i = 0; i < 6; i++) {
    float s = msg.data[i];
    uint16_t pwm_val = speedToPwm(s);
    // For DRI0002: you may need direction pin + enable (not shown here)
    // Here we just output PWM magnitude on PCA9685 channel
    pwm.setPWM(motorPwmChannel[i], 0, pwm_val);
    // Direction pins should be controlled via normal Arduino GPIO depending on sign(s)
    // TODO: add direction pin handling
  }
}

void steeringCmdCb(const std_msgs::Float32MultiArray &msg) {
  // Expect 4 elements: steering angles in degrees [-45..45]
  if (msg.data_length < 4) return;
  for (int i = 0; i < 4; i++) {
    float angle = msg.data[i];
    uint16_t pulse = angleToPulse(angle);
    pwm.setPWM(steeringServoChannel[i], 0, pulse);
  }
}

// ====== Setup ======
void setupEncoders() {
  for (int i = 0; i < 6; i++) {
    pinMode(encA[i], INPUT_PULLUP);
    pinMode(encB[i], INPUT_PULLUP);
  }
  attachInterrupt(digitalPinToInterrupt(encA[0]), encISR0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encA[1]), encISR1, CHANGE);
  // TODO: attach interrupts for remaining encoders as available
}

void setup() {
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50); // 50 Hz for servos / PWM

  setupEncoders();

  nh.initNode();
  nh.subscribe(sub_wheel_speed);
  nh.subscribe(sub_steering);

  wheel_ticks_msg.data = (int32_t*)malloc(6 * sizeof(int32_t));
  wheel_ticks_msg.data_length = 6;
  nh.advertise(pub_wheel_ticks);
}

// ====== Loop ======
unsigned long last_pub = 0;
const unsigned long PUB_PERIOD_MS = 50; // 20 Hz

void loop() {
  nh.spinOnce();

  unsigned long now = millis();
  if (now - last_pub >= PUB_PERIOD_MS) {
    noInterrupts();
    for (int i = 0; i < 6; i++) {
      wheel_ticks_msg.data[i] = (int32_t)wheelTicks[i];
    }
    interrupts();
    pub_wheel_ticks.publish(&wheel_ticks_msg);
    last_pub = now;
  }
}
