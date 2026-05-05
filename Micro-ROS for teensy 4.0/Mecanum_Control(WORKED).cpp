// ============================================================
//  micro-ROS Teensy 4.0 — Mecanum Drive (X-pattern)
//  แก้: ISR direction, wz normalize, Forward Kinematics wz
// ============================================================

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

// ── MOTOR PINS ── (แก้ตาม motor test จริง)
const int PWM_FL_A = 8,  PWM_FL_B = 9;   // pin 9/8 คือ FL, สลับ A/B เพื่อกลับทิศ
const int PWM_FR_A = 7,  PWM_FR_B = 6;   // pin 7/6 คือ FR ✅
const int PWM_RL_A = 4,  PWM_RL_B = 5;   // pin 5/4 คือ RL, สลับ A/B เพื่อกลับทิศ
const int PWM_RR_A = 3,  PWM_RR_B = 2;   // pin 3/2 คือ RR ✅

// ── ENCODER PINS ── (ให้ตรงกับ motor จริง)
const int ENC_FL_A = 16, ENC_FL_B = 17;  // FL ตรงกับ pin 9/8
const int ENC_FR_A = 10, ENC_FR_B = 11;  // FR ตรงกับ pin 7/6
const int ENC_RL_A = 14, ENC_RL_B = 15;  // RL ตรงกับ pin 5/4
const int ENC_RR_A = 12, ENC_RR_B = 13;  // RR ตรงกับ pin 3/2

#define LED_PIN 13

// ── ROBOT PARAMETERS ──
const float WHEEL_RADIUS  = 0.0625f;
const float WHEEL_BASE_LX = 0.205f;   // half-width  (m) ซ้าย↔ขวา
const float WHEEL_BASE_LY = 0.205f;   // half-length (m) หน้า↔หลัง
const float L             = WHEEL_BASE_LX + WHEEL_BASE_LY;
const int   TICKS_PER_REV = 360;
const float DIST_PER_TICK = (2.0f * M_PI * WHEEL_RADIUS) / TICKS_PER_REV;
const float MAX_SPEED     = 0.5f;     // m/s
const float MAX_ANGULAR   = 2.0f;     // rad/s  ← แยกจาก MAX_SPEED

// ── ENCODER TICKS ──
volatile long ticks_FL = 0, ticks_FR = 0;
volatile long ticks_RL = 0, ticks_RR = 0;
long prev_FL = 0, prev_FR = 0, prev_RL = 0, prev_RR = 0;

// ── ODOMETRY STATE ──
float odom_x = 0, odom_y = 0, odom_yaw = 0;
float odom_vx = 0, odom_vy = 0, odom_wz = 0;

// ── micro-ROS ──
rcl_subscription_t subscriber;
rcl_publisher_t    odom_publisher, tf_publisher;
rcl_timer_t        odom_timer;
geometry_msgs__msg__Twist            twist_msg;
nav_msgs__msg__Odometry              odom_msg;
tf2_msgs__msg__TFMessage             tf_msg;
geometry_msgs__msg__TransformStamped tf_stamped;
rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;

volatile float linear_vel_x = 0, linear_vel_y = 0, angular_vel = 0;
uint32_t debug_counter = 0;

typedef enum { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } State;
State state = WAITING_AGENT;

#define RCCHECK(fn)     { rcl_ret_t r = fn; if(r != RCL_RET_OK) return false; }
#define RCSOFTCHECK(fn) { rcl_ret_t r = fn; (void)r; }

// ── ISR ──────────────────────────────────────────────────────
// X-pattern: FL/RR หมุน forward = CW (มองจากนอก) → encoder B LOW ก่อน = +1
// ปรับ sign ถ้าล้อใดนับผิดทิศ (ดู debug ticks)
//
//  วิธีตรวจ: ดัน FL ไปข้างหน้าด้วยมือ → ticks_FL ควรเพิ่มขึ้น (+)
//            ถ้าลดลง → สลับ sign ของ isr_FL

// ── ISR ── (ล้อซ้ายกลับทิศแล้วจาก pin swap ข้างบน)
void isr_FL() { ticks_FL += (digitalRead(ENC_FL_B) == HIGH) ?  1 : -1; }
void isr_FR() { ticks_FR += (digitalRead(ENC_FR_B) == HIGH) ?  1 : -1; }
void isr_RL() { ticks_RL += (digitalRead(ENC_RL_B) == HIGH) ?  1 : -1; }
void isr_RR() { ticks_RR += (digitalRead(ENC_RR_B) == HIGH) ?  1 : -1; }

// ── MOTOR ──
void setMotor(int pwmA, int pwmB, float speed) {
  int pwm = constrain((int)(fabs(speed) * 255.0f), 0, 255);
  if      (speed >  0.01f) { analogWrite(pwmA, pwm); analogWrite(pwmB, 0);   }
  else if (speed < -0.01f) { analogWrite(pwmA, 0);   analogWrite(pwmB, pwm); }
  else                     { analogWrite(pwmA, 0);   analogWrite(pwmB, 0);   }
}

void stopAll() {
  setMotor(PWM_FL_A, PWM_FL_B, 0); setMotor(PWM_FR_A, PWM_FR_B, 0);
  setMotor(PWM_RL_A, PWM_RL_B, 0); setMotor(PWM_RR_A, PWM_RR_B, 0);
}

// ── QUATERNION ──
void yawToQuaternion(float yaw, double &qx, double &qy, double &qz, double &qw) {
  qx = 0.0; qy = 0.0;
  qz = sin(yaw * 0.5f);
  qw = cos(yaw * 0.5f);
}

// ── TIME ──
void getTimeNow(int32_t &sec, uint32_t &nanosec) {
  int64_t ms = rmw_uros_epoch_millis();
  if (ms > 0) { sec = ms/1000LL; nanosec = (ms%1000LL)*1000000ULL; }
  else { unsigned long m = millis(); sec = m/1000UL; nanosec = (m%1000UL)*1000000UL; }
}

// ── ODOM CALLBACK (50 Hz) ──
void odom_timer_callback(rcl_timer_t *timer, int64_t) {
  if (!timer) return;

  noInterrupts();
  long cFL=ticks_FL, cFR=ticks_FR, cRL=ticks_RL, cRR=ticks_RR;
  interrupts();

  // ระยะทาง (m) แต่ละล้อ — forward = ค่าบวกทุกล้อ
  float vFL = (cFL - prev_FL) * DIST_PER_TICK;
  float vFR = (cFR - prev_FR) * DIST_PER_TICK;
  float vRL = (cRL - prev_RL) * DIST_PER_TICK;
  float vRR = (cRR - prev_RR) * DIST_PER_TICK;
  prev_FL=cFL; prev_FR=cFR; prev_RL=cRL; prev_RR=cRR;

  // ── X-pattern Mecanum Forward Kinematics ──
  // vx  = ( FL + FR + RL + RR) / 4
  // vy  = (-FL + FR + RL - RR) / 4   [strafe right = +vy]
  // wz  = (-FL + FR - RL + RR) / (4 * lx)   [CCW = +wz]
  float raw_vx = ( vFL + vFR + vRL + vRR) * 0.25f;
  float raw_vy = (-vFL + vFR + vRL - vRR) * 0.25f;
  float raw_wz = (-vFL + vFR - vRL + vRR) / (4.0f * WHEEL_BASE_LX);

  // Mid-point integration (แม่นกว่า Euler ธรรมดา)
  float mid_yaw = odom_yaw + raw_wz * 0.5f;
  odom_yaw += raw_wz;
  odom_x   += raw_vx * cos(mid_yaw) - raw_vy * sin(mid_yaw);
  odom_y   += raw_vx * sin(mid_yaw) + raw_vy * cos(mid_yaw);

  const float dt    = 0.02f;
  const float ALPHA = 0.3f;
  odom_vx = ALPHA*(raw_vx/dt) + (1.0f-ALPHA)*odom_vx;
  odom_vy = ALPHA*(raw_vy/dt) + (1.0f-ALPHA)*odom_vy;
  odom_wz = ALPHA*(raw_wz/dt) + (1.0f-ALPHA)*odom_wz;
  if (fabs(odom_vx) < 0.001f) odom_vx = 0;
  if (fabs(odom_vy) < 0.001f) odom_vy = 0;
  if (fabs(odom_wz) < 0.001f) odom_wz = 0;

  double qx,qy,qz,qw;
  yawToQuaternion(odom_yaw, qx,qy,qz,qw);
  int32_t t_sec; uint32_t t_ns;
  getTimeNow(t_sec, t_ns);

  // DEBUG 1 วินาที — ดู sign ticks ว่าเดินหน้าแล้ว + ทุกล้อ
  if (++debug_counter >= 50) {
    debug_counter = 0;
    Serial.printf("FL:%ld FR:%ld RL:%ld RR:%ld | x:%.3f y:%.3f yaw:%.3f\n",
                  cFL, cFR, cRL, cRR, odom_x, odom_y, odom_yaw);
  }

  // Publish /odom
  odom_msg.header.stamp.sec = t_sec; odom_msg.header.stamp.nanosec = t_ns;
  odom_msg.pose.pose.position.x = odom_x;
  odom_msg.pose.pose.position.y = odom_y;
  odom_msg.pose.pose.position.z = 0;
  odom_msg.pose.pose.orientation.x = qx; odom_msg.pose.pose.orientation.y = qy;
  odom_msg.pose.pose.orientation.z = qz; odom_msg.pose.pose.orientation.w = qw;
  odom_msg.twist.twist.linear.x  = odom_vx;
  odom_msg.twist.twist.linear.y  = odom_vy;
  odom_msg.twist.twist.angular.z = odom_wz;
  RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));

  // Publish /tf
  tf_stamped.header.stamp.sec = t_sec; tf_stamped.header.stamp.nanosec = t_ns;
  tf_stamped.transform.translation.x = odom_x;
  tf_stamped.transform.translation.y = odom_y;
  tf_stamped.transform.translation.z = 0;
  tf_stamped.transform.rotation.x = qx; tf_stamped.transform.rotation.y = qy;
  tf_stamped.transform.rotation.z = qz; tf_stamped.transform.rotation.w = qw;
  RCSOFTCHECK(rcl_publish(&tf_publisher, &tf_msg, NULL));
}

// ── CMD_VEL CALLBACK ──
void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  linear_vel_x = msg->linear.x;
  linear_vel_y = msg->linear.y;
  angular_vel  = msg->angular.z;

  // Normalize แยก linear/angular ถูกหน่วย
  float vx = linear_vel_x / MAX_SPEED;
  float vy = linear_vel_y / MAX_SPEED;
  float wz = angular_vel  / MAX_ANGULAR;  // ✅ rad/s หาร rad/s

  // X-pattern Mecanum Inverse Kinematics
  // FL =  vx - vy - wz
  // FR =  vx + vy + wz
  // RL =  vx + vy - wz
  // RR =  vx - vy + wz
  float fl =  vx - vy - wz;
  float fr =  vx + vy + wz;
  float rl =  vx + vy - wz;
  float rr =  vx - vy + wz;

  // Normalize ถ้าเกิน 1.0
  float mx = fabs(fl);
  if (fabs(fr)>mx) mx=fabs(fr);
  if (fabs(rl)>mx) mx=fabs(rl);
  if (fabs(rr)>mx) mx=fabs(rr);
  if (mx > 1.0f) { fl/=mx; fr/=mx; rl/=mx; rr/=mx; }

  setMotor(PWM_FL_A, PWM_FL_B, fl);
  setMotor(PWM_FR_A, PWM_FR_B, fr);
  setMotor(PWM_RL_A, PWM_RL_B, rl);
  setMotor(PWM_RR_A, PWM_RR_B, rr);
}

// ── CREATE / DESTROY ENTITIES ──
bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "teensy_node", "", &support));
  rmw_uros_sync_session(1000);

  RCCHECK(rclc_subscription_init_default(&subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  RCCHECK(rclc_publisher_init_default(&odom_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom"));
  RCCHECK(rclc_publisher_init_default(&tf_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage), "tf"));
  RCCHECK(rclc_timer_init_default(&odom_timer, &support, RCL_MS_TO_NS(20), odom_timer_callback));
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &twist_msg, &cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &odom_timer));

  static char of[] = "odom", oc[] = "base_link";
  odom_msg.header.frame_id.data = of; odom_msg.header.frame_id.size = strlen(of);
  odom_msg.child_frame_id.data  = oc; odom_msg.child_frame_id.size  = strlen(oc);

  tf_msg.transforms.data = &tf_stamped; tf_msg.transforms.size = 1; tf_msg.transforms.capacity = 1;
  static char tf[] = "odom", tc[] = "base_link";
  tf_stamped.header.frame_id.data = tf; tf_stamped.header.frame_id.size = strlen(tf);
  tf_stamped.child_frame_id.data  = tc; tf_stamped.child_frame_id.size  = strlen(tc);

  noInterrupts();
  prev_FL=ticks_FL; prev_FR=ticks_FR; prev_RL=ticks_RL; prev_RR=ticks_RR;
  interrupts();
  return true;
}

void destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  rcl_subscription_fini(&subscriber, &node);
  rcl_publisher_fini(&odom_publisher, &node);
  rcl_publisher_fini(&tf_publisher, &node);
  rcl_timer_fini(&odom_timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// ── SETUP ──
void setup() {
  Serial.begin(115200);
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, LOW);

  pinMode(PWM_FL_A,OUTPUT); pinMode(PWM_FL_B,OUTPUT);
  pinMode(PWM_FR_A,OUTPUT); pinMode(PWM_FR_B,OUTPUT);
  pinMode(PWM_RL_A,OUTPUT); pinMode(PWM_RL_B,OUTPUT);
  pinMode(PWM_RR_A,OUTPUT); pinMode(PWM_RR_B,OUTPUT);

  pinMode(ENC_FL_A,INPUT_PULLUP); pinMode(ENC_FL_B,INPUT_PULLUP);
  pinMode(ENC_FR_A,INPUT_PULLUP); pinMode(ENC_FR_B,INPUT_PULLUP);
  pinMode(ENC_RL_A,INPUT_PULLUP); pinMode(ENC_RL_B,INPUT_PULLUP);
  pinMode(ENC_RR_A,INPUT_PULLUP); pinMode(ENC_RR_B,INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_FL_A), isr_FL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_FR_A), isr_FR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RL_A), isr_RL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RR_A), isr_RR, CHANGE);
}

// ── LOOP ──
// ── LOOP ── แก้ case AGENT_AVAILABLE และ AGENT_CONNECTED
void loop() {
  switch (state) {
    case WAITING_AGENT:
      if (rmw_uros_ping_agent(500, 1) == RMW_RET_OK) state = AGENT_AVAILABLE;
      break;

    case AGENT_AVAILABLE:
      // ✅ แก้: ใช้ if/else แทน ternary + else
      if (create_entities()) {
        digitalWrite(LED_PIN, HIGH);
        state = AGENT_CONNECTED;
      } else {
        destroy_entities();
        state = WAITING_AGENT;
      }
      break;

    case AGENT_CONNECTED:
      // ✅ แก้: แยก if/else ออกมาชัดๆ
      if (rmw_uros_ping_agent(100, 3) == RMW_RET_OK) {
        RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
      } else {
        state = AGENT_DISCONNECTED;
      }
      break;

    case AGENT_DISCONNECTED:
      stopAll();
      linear_vel_x = 0; linear_vel_y = 0; angular_vel = 0;
      destroy_entities();
      digitalWrite(LED_PIN, LOW);
      state = WAITING_AGENT;
      break;

    default:
      break;
  }
}
