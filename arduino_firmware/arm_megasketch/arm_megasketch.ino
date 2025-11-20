#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>

ros::NodeHandle nh;

// Joint configuration
const int NUM_JOINTS = 5;

const uint8_t STEP_PIN[NUM_JOINTS] = {22, 24, 26, 28, 30};
const uint8_t DIR_PIN[NUM_JOINTS]  = {23, 25, 27, 29, 31};
const uint8_t LIM_PIN[NUM_JOINTS]  = {32, 33, 34, 35, 36};
const uint8_t ENABLE_PIN = 37;

// Steps per radian for each joint
float steps_per_rad[NUM_JOINTS] = {
  200.0 * 16.0 / (2 * 3.14159),  // example: 200 steps/rev, 1:1, 16x microstepping
  200.0 * 16.0 / (2 * 3.14159),
  200.0 * 16.0 / (2 * 3.14159),
  200.0 * 16.0 / (2 * 3.14159),
  200.0 * 16.0 / (2 * 3.14159)
};

volatile long joint_steps[NUM_JOINTS] = {0, 0, 0, 0, 0};
float target_rad[NUM_JOINTS] = {0, 0, 0, 0, 0};
bool homed = false;

// Publishers / Subscribers
sensor_msgs::JointState js_msg;
ros::Publisher pub_joint_states("arm/joint_states", &js_msg);

void jointCmdCb(const sensor_msgs::JointState &msg) {
  int n = min((int)msg.position_length, NUM_JOINTS);
  for (int i = 0; i < n; i++) {
    target_rad[i] = msg.position[i];
  }
}
ros::Subscriber<sensor_msgs::JointState> sub_cmd("arm/joint_command", &jointCmdCb);

// Homing service
bool do_home(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  digitalWrite(ENABLE_PIN, LOW); // enable drivers
  for (int j = 0; j < NUM_JOINTS; j++) {
    // Move negative until limit switch is hit
    digitalWrite(DIR_PIN[j], LOW);
    while (digitalRead(LIM_PIN[j]) == HIGH) { // active low switch?
      digitalWrite(STEP_PIN[j], HIGH);
      delayMicroseconds(800);
      digitalWrite(STEP_PIN[j], LOW);
      delayMicroseconds(800);
    }
    // Set zero
    joint_steps[j] = 0;
    target_rad[j] = 0.0;
  }
  homed = true;
  res.success = true;
  res.message = "Arm homed";
  return true;
}
ros::ServiceServer<std_srvs::Trigger::Request, std_srvs::Trigger::Response> srv_home("arm/home", &do_home);

void setup() {
  for (int j = 0; j < NUM_JOINTS; j++) {
    pinMode(STEP_PIN[j], OUTPUT);
    pinMode(DIR_PIN[j], OUTPUT);
    pinMode(LIM_PIN[j], INPUT_PULLUP);
  }
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH); // disable initially

  nh.initNode();
  nh.advertise(pub_joint_states);
  nh.subscribe(sub_cmd);
  nh.advertiseService(srv_home);

  js_msg.name_length = NUM_JOINTS;
  js_msg.position_length = NUM_JOINTS;
  js_msg.velocity_length = NUM_JOINTS;
  js_msg.effort_length = NUM_JOINTS;
  js_msg.name = (char**)malloc(NUM_JOINTS * sizeof(char*));
  js_msg.position = (float*)malloc(NUM_JOINTS * sizeof(float));
  js_msg.velocity = (float*)malloc(NUM_JOINTS * sizeof(float));
  js_msg.effort = (float*)malloc(NUM_JOINTS * sizeof(float));

  // Names (must be static strings)
  js_msg.name[0] = (char*)"joint_1";
  js_msg.name[1] = (char*)"joint_2";
  js_msg.name[2] = (char*)"joint_3";
  js_msg.name[3] = (char*)"joint_4";
  js_msg.name[4] = (char*)"joint_5";
}

unsigned long last_pub = 0;
const unsigned long PUB_PERIOD_MS = 50;

void loop() {
  nh.spinOnce();

  if (homed) {
    digitalWrite(ENABLE_PIN, LOW); // enable
    // Simple synchronous stepping; for better control use Bresenham / motion planner.
    for (int j = 0; j < NUM_JOINTS; j++) {
      long target_steps = (long)(target_rad[j] * steps_per_rad[j]);
      long diff = target_steps - joint_steps[j];
      if (diff != 0) {
        bool dir = diff > 0;
        digitalWrite(DIR_PIN[j], dir ? HIGH : LOW);
        digitalWrite(STEP_PIN[j], HIGH);
        delayMicroseconds(800);
        digitalWrite(STEP_PIN[j], LOW);
        delayMicroseconds(800);
        joint_steps[j] += dir ? 1 : -1;
      }
    }
  } else {
    digitalWrite(ENABLE_PIN, HIGH); // disable until homed
  }

  unsigned long now = millis();
  if (now - last_pub >= PUB_PERIOD_MS) {
    js_msg.header.stamp = nh.now();
    for (int j = 0; j < NUM_JOINTS; j++) {
      js_msg.position[j] = (float)(joint_steps[j] / steps_per_rad[j]);
      js_msg.velocity[j] = 0.0f;
      js_msg.effort[j] = 0.0f;
    }
    pub_joint_states.publish(&js_msg);
    last_pub = now;
  }
}
