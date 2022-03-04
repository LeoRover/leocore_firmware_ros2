#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <rmw_microros/rmw_microros.h>
#include <uxr/client/transport.h>

#include <std_msgs/msg/float32.h>

#include "wheel_controller.hpp"

#include "mainf.h"

#include "microros/uart_transport.h"

#include "firmware/configuration.hpp"
#include "firmware/imu_receiver.hpp"
#include "firmware/parameters.hpp"

static rclc_support_t support;
static rcl_node_t node;
static bool configured = false;

static std_msgs__msg__Float32 battery;
static std_msgs__msg__Float32 battery_averaged;
static rcl_publisher_t battery_pub;
static rcl_publisher_t battery_averaged_pub;
static CircularBuffer<float> battery_buffer_(BATTERY_BUFFER_SIZE);
static bool publish_battery = false;

rclc_executor_t executor;

Parameters params;

void error_loop() {
  for (;;) {
  }
}

void setup() {
  rmw_uros_set_uart_transport(&huart1);

  static rcl_allocator_t allocator = rcl_get_default_allocator();

  rcl_ret_t rc = rclc_support_init(&support, 0, NULL, &allocator);
  if (rc != RCL_RET_OK) error_loop();

  rc = rclc_node_init_default(&node, "firmware", "", &support);
  if (rc != RCL_RET_OK) error_loop();

  rclc_publisher_init_default(
      &battery_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "firmware/battery");
  rclc_publisher_init_default(
      &battery_averaged_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "firmware/battery_averaged");

  rclc_executor_init(&executor, &support.context, 1, &allocator);

  configured = true;
}

void loop() {
  rcl_ret_t rc = rclc_executor_spin_some(&executor, 1000 * 1000 * 1000);
  if (rc != RCL_RET_OK) error_loop();

  if (publish_battery) {
    rcl_publish(&battery_pub, &battery, NULL);
    rcl_publish(&battery_averaged_pub, &battery_averaged, NULL);
    publish_battery = false;
  }
}

void update() {
  static uint32_t cnt = 0;
  ++cnt;

  static float battery_sum = 0.0F;
  static float battery_avg = 0.0F;
  float battery_new = static_cast<float>(BATTERY_ADC) * BATTERY_ADC_TO_VOLTAGE;
  battery_sum += battery_new;
  battery_sum -= battery_buffer_.push_back(battery_new);
  battery_avg =
      battery_sum / static_cast<float>(std::min(BATTERY_BUFFER_SIZE, cnt));

  if (battery_avg < params.battery_min_voltage) {
    if (cnt % 10 == 0) gpio_toggle(LED);
  } else {
    gpio_reset(LED);
  }

  if (!configured) return;

  if (cnt % BATTERY_PUB_PERIOD == 0 && !publish_battery) {
    battery.data = battery_new;
    battery_averaged.data = battery_avg;

    publish_battery = true;
  }
}