#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <rmw_microros/rmw_microros.h>
#include <uxr/client/transport.h>

#include <std_msgs/msg/int32.h>

#include "mainf.h"
#include "usart.h"

#include "microros/uart_transport.h"

rclc_support_t support;
rcl_node_t node;
rcl_publisher_t publisher;
rclc_executor_t executor;
rcl_timer_t timer;

std_msgs__msg__Int32 msg;

void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    rcl_publish(&publisher, &msg, NULL);
    msg.data++;
  }
}

void error_loop() {
  for (;;) {
  }
}

#define RCCHECK(fn)                \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      error_loop();                \
    }                              \
  }

void setup() {
  rmw_uros_set_uart_transport(&huart1);

  static rcl_allocator_t allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(&node, "firmware", "", &support));

  RCCHECK(rclc_publisher_init_default(
      &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "test"));

  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(1000),
                                  timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg.data = 0;
}

void loop() {
  RCCHECK(rclc_executor_spin(&executor));
}