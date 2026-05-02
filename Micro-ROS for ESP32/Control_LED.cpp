#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>

#define LED_PIN 2

// เพิ่ม timeout ping ให้นานขึ้น และลด attempt
#define PING_TIMEOUT_MS   1000   // รอ agent ตอบ 1000ms (เดิมอาจสั้นเกินไป)
#define PING_ATTEMPTS     5      // ลองหลายครั้งก่อนตัดสินว่า disconnect

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_subscription_t subscriber;
rclc_executor_t executor;
std_msgs__msg__Bool msg;

enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
AgentState state = WAITING_AGENT;

void subscription_callback(const void * msgin)
{
  const std_msgs__msg__Bool * m = (const std_msgs__msg__Bool *)msgin;
  digitalWrite(LED_PIN, m->data ? HIGH : LOW);
}

bool create_entities()
{
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_node", "", &support));
  RCCHECK(rclc_subscription_init_default(
    &subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "led_cmd"
  ));
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &subscriber, &msg,
    &subscription_callback, ON_NEW_DATA
  ));
  return true;
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  rcl_subscription_fini(&subscriber, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  // ★ สำคัญ: รอให้ serial port เสถียรก่อน
  delay(3000);
}

void loop()
{
  switch (state) {
    case WAITING_AGENT:
      // ping นานขึ้น + หลาย attempt → ลด false positive
      if (rmw_uros_ping_agent(PING_TIMEOUT_MS, PING_ATTEMPTS) == RMW_RET_OK) {
        state = AGENT_AVAILABLE;
        Serial.println("[INFO] Agent found!");
      } else {
        delay(500); // ★ อย่าวนถี่เกินไปขณะรอ
      }
      break;

    case AGENT_AVAILABLE:
      if (create_entities()) {
        state = AGENT_CONNECTED;
        Serial.println("[INFO] Connected!");
      } else {
        destroy_entities();
        state = WAITING_AGENT;
        Serial.println("[ERROR] create_entities failed, retrying...");
        delay(1000);
      }
      break;

    case AGENT_CONNECTED:
      // ★ spin ก่อน แล้วค่อย ping — อย่า ping ทุก loop
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

      // ping ทุก 2 วินาที แทนที่จะทุก loop
      {
        static unsigned long last_ping = 0;
        if (millis() - last_ping > 2000) {
          last_ping = millis();
          if (rmw_uros_ping_agent(PING_TIMEOUT_MS, PING_ATTEMPTS) != RMW_RET_OK) {
            state = AGENT_DISCONNECTED;
            Serial.println("[WARN] Agent lost!");
          }
        }
      }
      break;

    case AGENT_DISCONNECTED:
      destroy_entities();
      digitalWrite(LED_PIN, LOW);
      state = WAITING_AGENT;
      Serial.println("[INFO] Reconnecting...");
      delay(1000); // ★ รอก่อน reconnect
      break;
  }
}
