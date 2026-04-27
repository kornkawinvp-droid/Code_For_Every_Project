// ============================================================
//  micro-ROS Teensy 4.0 — Teleop + Odometry + TF
//  รับ cmd_vel → คุมมอเตอร์
//  publish /odom (nav_msgs/Odometry)
//  publish /tf   (tf2_msgs/TFMessage)
//  คำนวณ odom จาก encoder 4 ล้อ (differential drive)
// ============================================================

// ── 1. LIBRARIES ─────────────────────────────────────────────
#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <math.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <tf2_msgs/msg/tf_message.h>
#include <geometry_msgs/msg/transform_stamped.h>

// ── 2. MOTOR PINS ────────────────────────────────────────────
const int PWM_LEFT_A   = 5;
const int PWM_LEFT_B   = 4;
const int PWM_RIGHT_A  = 3;
const int PWM_RIGHT_B  = 2;
const int PWM_LEFT2_A  = 9;
const int PWM_LEFT2_B  = 8;
const int PWM_RIGHT2_A = 7;
const int PWM_RIGHT2_B = 6;

// ── 3. ENCODER PINS ─────────────────────────────────────────
const int ENC_RR_A = 10;
const int ENC_RR_B = 11;
const int ENC_RF_A = 12;
const int ENC_RF_B = 13;
const int ENC_LF_A = 14;
const int ENC_LF_B = 15;
const int ENC_LR_A = 16;
const int ENC_LR_B = 17;

#define LED_PIN 13   // NOTE: LED_PIN ซ้ำกับ ENC_RF_B — ถ้าใช้ encoder pin 13 จริงให้เปลี่ยน LED เป็น pin อื่น

// ── 4. ROBOT PARAMETERS ─────────────────────────────────────
const float WHEEL_RADIUS      = 0.05f;    // เมตร — ปรับตามล้อจริง
const float WHEEL_BASE        = 0.30f;    // เมตร ระยะห่างซ้าย-ขวา — ปรับตามรถจริง
const int   TICKS_PER_REV     = 360;      // ticks ต่อรอบ — ปรับตาม encoder จริง
const float DIST_PER_TICK     = (2.0f * M_PI * WHEEL_RADIUS) / TICKS_PER_REV;
const float MAX_SPEED         = 0.5f;     // m/s

// ── 5. ENCODER TICKS (volatile) ─────────────────────────────
volatile long ticks_RR = 0;
volatile long ticks_RF = 0;
volatile long ticks_LF = 0;
volatile long ticks_LR = 0;

// snapshot ของรอบก่อน
long prev_ticks_left  = 0;   // เฉลี่ย LF + LR
long prev_ticks_right = 0;   // เฉลี่ย RF + RR

// ── 6. ODOMETRY STATE ────────────────────────────────────────
float odom_x   = 0.0f;
float odom_y   = 0.0f;
float odom_yaw = 0.0f;
float odom_vx  = 0.0f;
float odom_wz  = 0.0f;

// ── 7. micro-ROS OBJECTS ─────────────────────────────────────
rcl_subscription_t subscriber;
rcl_publisher_t    odom_publisher;
rcl_publisher_t    tf_publisher;
rcl_timer_t        odom_timer;

geometry_msgs__msg__Twist         twist_msg;
nav_msgs__msg__Odometry           odom_msg;
tf2_msgs__msg__TFMessage          tf_msg;
geometry_msgs__msg__TransformStamped tf_stamped;

rclc_executor_t  executor;
rclc_support_t   support;
rcl_allocator_t  allocator;
rcl_node_t       node;

volatile float linear_vel  = 0.0f;
volatile float angular_vel = 0.0f;

// ── 8. STATE MACHINE ─────────────────────────────────────────
typedef enum {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} State;

State state = WAITING_AGENT;

// ── 9. ERROR MACROS ──────────────────────────────────────────
#define RCCHECK(fn)     { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// ── 10. ENCODER ISRs ─────────────────────────────────────────
// IRAM_ATTR ใช้กับ ESP32 เท่านั้น — Teensy ไม่ต้องการ
void isr_RR() {
  ticks_RR += (digitalRead(ENC_RR_B) == HIGH) ? 1 : -1;
}
void isr_RF() {
  ticks_RF += (digitalRead(ENC_RF_B) == HIGH) ? 1 : -1;
}
void isr_LF() {
  ticks_LF += (digitalRead(ENC_LF_B) == HIGH) ? -1 : 1;  // ด้านซ้ายกลับทิศ
}
void isr_LR() {
  ticks_LR += (digitalRead(ENC_LR_B) == HIGH) ? -1 : 1;
}

// ── 11. MOTOR FUNCTION ───────────────────────────────────────
void setMotor(int pwmA, int pwmB, float speed) {
  int pwm = (int)(fabs(speed) * 255.0f);
  if (pwm > 255) pwm = 255;

  if (speed > 0.01f) {
    analogWrite(pwmA, pwm);
    analogWrite(pwmB, 0);
  } else if (speed < -0.01f) {
    analogWrite(pwmA, 0);
    analogWrite(pwmB, pwm);
  } else {
    analogWrite(pwmA, 0);
    analogWrite(pwmB, 0);
  }
}

// ── 12. QUATERNION FROM YAW ──────────────────────────────────
void yawToQuaternion(float yaw,
                     double &qx, double &qy, double &qz, double &qw) {
  qx = 0.0;
  qy = 0.0;
  qz = sin(yaw * 0.5f);
  qw = cos(yaw * 0.5f);
}

// ── 13. GET TIME (microseconds → sec + nanosec) ──────────────
void getTimeNow(int32_t &sec, uint32_t &nanosec) {
  // ใช้ rmw_uros_sync_session เพื่อให้นาฬิกาตรงกับ ROS agent
  // ถ้า sync แล้ว ใช้ rmw_uros_epoch_millis
  int64_t ms = rmw_uros_epoch_millis();
  if (ms > 0) {
    sec     = (int32_t)(ms / 1000LL);
    nanosec = (uint32_t)((ms % 1000LL) * 1000000ULL);
  } else {
    // fallback: millis() จาก Arduino
    unsigned long m = millis();
    sec     = (int32_t)(m / 1000UL);
    nanosec = (uint32_t)((m % 1000UL) * 1000000UL);
  }
}

// ── 14. ODOM TIMER CALLBACK (50 Hz) ─────────────────────────
void odom_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer == NULL) return;

  // ── อ่าน encoder อย่างปลอดภัย ──────────
  noInterrupts();
  long cur_LF = ticks_LF;
  long cur_LR = ticks_LR;
  long cur_RF = ticks_RF;
  long cur_RR = ticks_RR;
  interrupts();

  // เฉลี่ย 2 ล้อต่อด้าน
  long cur_left  = (cur_LF + cur_LR) / 2;
  long cur_right = (cur_RF + cur_RR) / 2;

  long delta_left  = cur_left  - prev_ticks_left;
  long delta_right = cur_right - prev_ticks_right;

  prev_ticks_left  = cur_left;
  prev_ticks_right = cur_right;

  // ── คำนวณระยะทาง ────────────────────────
  float dist_left  = delta_left  * DIST_PER_TICK;
  float dist_right = delta_right * DIST_PER_TICK;
  float dist_center = (dist_left + dist_right) * 0.5f;
  float delta_yaw   = (dist_right - dist_left) / WHEEL_BASE;

  // dt ≈ 20ms (50Hz)
  const float dt = 0.02f;

  // ── อัปเดต pose ─────────────────────────
  odom_yaw += delta_yaw;
  odom_x   += dist_center * cos(odom_yaw);
  odom_y   += dist_center * sin(odom_yaw);

  // velocity
  odom_vx = dist_center / dt;
  odom_wz = delta_yaw   / dt;

  // ── Quaternion ───────────────────────────
  double qx, qy, qz, qw;
  yawToQuaternion(odom_yaw, qx, qy, qz, qw);

  // ── Timestamp ────────────────────────────
  int32_t  t_sec;
  uint32_t t_nanosec;
  getTimeNow(t_sec, t_nanosec);

  // ════════════════════════════════════════
  //  PUBLISH /odom
  // ════════════════════════════════════════
  odom_msg.header.stamp.sec     = t_sec;
  odom_msg.header.stamp.nanosec = t_nanosec;

  // frame ids — ตั้งค่าครั้งเดียวใน create_entities()
  // ที่นี่แค่อัปเดตข้อมูล

  odom_msg.pose.pose.position.x  = odom_x;
  odom_msg.pose.pose.position.y  = odom_y;
  odom_msg.pose.pose.position.z  = 0.0;

  odom_msg.pose.pose.orientation.x = qx;
  odom_msg.pose.pose.orientation.y = qy;
  odom_msg.pose.pose.orientation.z = qz;
  odom_msg.pose.pose.orientation.w = qw;

  odom_msg.twist.twist.linear.x  = odom_vx;
  odom_msg.twist.twist.angular.z = odom_wz;

  RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));

  // ════════════════════════════════════════
  //  PUBLISH /tf  (odom → base_link)
  // ════════════════════════════════════════
  tf_stamped.header.stamp.sec     = t_sec;
  tf_stamped.header.stamp.nanosec = t_nanosec;

  tf_stamped.transform.translation.x = odom_x;
  tf_stamped.transform.translation.y = odom_y;
  tf_stamped.transform.translation.z = 0.0;

  tf_stamped.transform.rotation.x = qx;
  tf_stamped.transform.rotation.y = qy;
  tf_stamped.transform.rotation.z = qz;
  tf_stamped.transform.rotation.w = qw;

  RCSOFTCHECK(rcl_publish(&tf_publisher, &tf_msg, NULL));
}

// ── 15. CMD_VEL CALLBACK ─────────────────────────────────────
void cmd_vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg =
      (const geometry_msgs__msg__Twist *)msgin;

  linear_vel  = msg->linear.x;
  angular_vel = msg->angular.z;

  float left_speed  = -(linear_vel - angular_vel) / MAX_SPEED;
  float right_speed =  (linear_vel + angular_vel) / MAX_SPEED;

  if (left_speed  >  1.0f) left_speed  =  1.0f;
  if (left_speed  < -1.0f) left_speed  = -1.0f;
  if (right_speed >  1.0f) right_speed =  1.0f;
  if (right_speed < -1.0f) right_speed = -1.0f;

  setMotor(PWM_LEFT_A,   PWM_LEFT_B,   left_speed);
  setMotor(PWM_RIGHT_A,  PWM_RIGHT_B,  right_speed);
  setMotor(PWM_LEFT2_A,  PWM_LEFT2_B,  left_speed);
  setMotor(PWM_RIGHT2_A, PWM_RIGHT2_B, right_speed);
}

// ── 16. CREATE ENTITIES ─────────────────────────────────────
bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "teensy_node", "", &support));

  // ── sync นาฬิกากับ agent ──────────────
  rmw_uros_sync_session(1000);

  // ── subscriber: /cmd_vel ──────────────
  RCCHECK(rclc_subscription_init_default(
    &subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // ── publisher: /odom ──────────────────
  RCCHECK(rclc_publisher_init_default(
    &odom_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom"));

  // ── publisher: /tf ────────────────────
  RCCHECK(rclc_publisher_init_default(
    &tf_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
    "tf"));

  // ── timer 20ms = 50Hz ─────────────────
  RCCHECK(rclc_timer_init_default(
    &odom_timer, &support,
    RCL_MS_TO_NS(20),
    odom_timer_callback));

  // ── executor: 1 sub + 1 timer = 2 handles ──
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &subscriber, &twist_msg,
    &cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &odom_timer));

  // ── ตั้งค่า static fields ─────────────
  // odom header
  static char odom_frame_id[]    = "odom";
  static char odom_child_id[]    = "base_link";
  odom_msg.header.frame_id.data  = odom_frame_id;
  odom_msg.header.frame_id.size  = strlen(odom_frame_id);
  odom_msg.child_frame_id.data   = odom_child_id;
  odom_msg.child_frame_id.size   = strlen(odom_child_id);

  // tf TFMessage ต้องชี้ไปที่ array ของ TransformStamped
  tf_msg.transforms.data  = &tf_stamped;
  tf_msg.transforms.size  = 1;
  tf_msg.transforms.capacity = 1;

  // tf_stamped header
  static char tf_frame_id[]          = "odom";
  static char tf_child_frame_id[]    = "base_link";
  tf_stamped.header.frame_id.data    = tf_frame_id;
  tf_stamped.header.frame_id.size    = strlen(tf_frame_id);
  tf_stamped.child_frame_id.data     = tf_child_frame_id;
  tf_stamped.child_frame_id.size     = strlen(tf_child_frame_id);

  return true;
}

// ── 17. DESTROY ENTITIES ─────────────────────────────────────
void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  (void) rcl_subscription_fini(&subscriber, &node);
  (void) rcl_publisher_fini(&odom_publisher, &node);
  (void) rcl_publisher_fini(&tf_publisher,   &node);
  (void) rcl_timer_fini(&odom_timer);
  rclc_executor_fini(&executor);
  (void) rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// ── 18. SETUP ────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // motor pins
  pinMode(PWM_LEFT_A,   OUTPUT);
  pinMode(PWM_LEFT_B,   OUTPUT);
  pinMode(PWM_RIGHT_A,  OUTPUT);
  pinMode(PWM_RIGHT_B,  OUTPUT);
  pinMode(PWM_LEFT2_A,  OUTPUT);
  pinMode(PWM_LEFT2_B,  OUTPUT);
  pinMode(PWM_RIGHT2_A, OUTPUT);
  pinMode(PWM_RIGHT2_B, OUTPUT);

  // encoder pins
  pinMode(ENC_RR_A, INPUT_PULLUP);
  pinMode(ENC_RR_B, INPUT_PULLUP);
  pinMode(ENC_RF_A, INPUT_PULLUP);
  pinMode(ENC_RF_B, INPUT_PULLUP);
  pinMode(ENC_LF_A, INPUT_PULLUP);
  pinMode(ENC_LF_B, INPUT_PULLUP);
  pinMode(ENC_LR_A, INPUT_PULLUP);
  pinMode(ENC_LR_B, INPUT_PULLUP);

  // attach interrupts บน channel A ของแต่ละ encoder
  attachInterrupt(digitalPinToInterrupt(ENC_RR_A), isr_RR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RF_A), isr_RF, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_LF_A), isr_LF, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_LR_A), isr_LR, CHANGE);
}

// ── 19. LOOP ─────────────────────────────────────────────────
void loop() {
  switch (state) {

    case WAITING_AGENT:
      if (rmw_uros_ping_agent(500, 1) == RMW_RET_OK) {
        state = AGENT_AVAILABLE;
      }
      break;

    case AGENT_AVAILABLE:
      if (create_entities()) {
        digitalWrite(LED_PIN, HIGH);
        state = AGENT_CONNECTED;
      } else {
        destroy_entities();
        state = WAITING_AGENT;
      }
      break;

    case AGENT_CONNECTED:
      if (rmw_uros_ping_agent(100, 3) == RMW_RET_OK) {
        RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
      } else {
        state = AGENT_DISCONNECTED;
      }
      break;

    case AGENT_DISCONNECTED:
      setMotor(PWM_LEFT_A,   PWM_LEFT_B,   0);
      setMotor(PWM_RIGHT_A,  PWM_RIGHT_B,  0);
      setMotor(PWM_LEFT2_A,  PWM_LEFT2_B,  0);
      setMotor(PWM_RIGHT2_A, PWM_RIGHT2_B, 0);

      linear_vel  = 0.0f;
      angular_vel = 0.0f;

      destroy_entities();
      digitalWrite(LED_PIN, LOW);
      state = WAITING_AGENT;
      break;

    default:
      break;
  }
}
