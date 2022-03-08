#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rosidl_runtime_c/string_functions.h>

#include <rmw_microros/rmw_microros.h>
#include <uxr/client/transport.h>

#include <geometry_msgs/msg/twist.h>
#include <leo_msgs/msg/imu.h>
#include <leo_msgs/msg/wheel_odom.h>
#include <leo_msgs/msg/wheel_states.h>
#include <std_msgs/msg/float32.h>
#include <std_srvs/srv/trigger.h>

#include "wheel_controller.hpp"

#include "mainf.h"

#include "microros/uart_transport.h"

#include "firmware/configuration.hpp"
#include "firmware/imu_receiver.hpp"
#include "firmware/parameters.hpp"

static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;
static bool configured = false;

static std_msgs__msg__Float32 battery;
static std_msgs__msg__Float32 battery_averaged;
static rcl_publisher_t battery_pub;
static rcl_publisher_t battery_averaged_pub;
static CircularBuffer<float> battery_buffer_(BATTERY_BUFFER_SIZE);
static bool publish_battery = false;

static leo_msgs__msg__WheelOdom wheel_odom;
static rcl_publisher_t wheel_odom_pub;
static bool publish_wheel_odom = false;

static leo_msgs__msg__WheelStates wheel_states;
static rcl_publisher_t wheel_states_pub;
static bool publish_wheel_states = false;

static leo_msgs__msg__Imu imu;
static rcl_publisher_t imu_pub;
static bool publish_imu = false;

static rcl_subscription_t twist_sub;
static geometry_msgs__msg__Twist twist_msg;

#define WHEEL_WRAPPER(NAME)                         \
  static const char* NAME##_cmd_pwm_topic =         \
      "firmware/wheel_" #NAME "/cmd_pwm_duty";      \
  static const char* NAME##_cmd_vel_topic =         \
      "firmware/wheel_" #NAME "/cmd_velocity";      \
  static rcl_subscription_t NAME##_cmd_pwm_sub;     \
  static rcl_subscription_t NAME##_cmd_vel_sub;     \
  static std_msgs__msg__Float32 NAME##_cmd_pwm_msg; \
  static std_msgs__msg__Float32 NAME##_cmd_vel_msg;

WHEEL_WRAPPER(FL)
WHEEL_WRAPPER(RL)
WHEEL_WRAPPER(FR)
WHEEL_WRAPPER(RR)

static rcl_service_t reset_odometry_srv, firmware_version_srv, board_type_srv,
    reset_board_srv;
static std_srvs__srv__Trigger_Request reset_odometry_req, firmware_version_req,
    board_type_req, reset_board_req;
static std_srvs__srv__Trigger_Response reset_odometry_res, firmware_version_res,
    board_type_res, reset_board_res;

static bool reset_request = false;

MotorController MotA(MOT_A_CONFIG);
MotorController MotB(MOT_B_CONFIG);
MotorController MotC(MOT_C_CONFIG);
MotorController MotD(MOT_D_CONFIG);

static DiffDriveController dc(DD_CONFIG);
static ImuReceiver imu_receiver(&IMU_I2C);

static Parameters params;

static void error_loop() {
  for (;;) {
  }
}

static void cmdVelCallback(const void* msgin) {
  const geometry_msgs__msg__Twist* msg =
      (const geometry_msgs__msg__Twist*)msgin;
  dc.setSpeed(msg->linear.x, msg->angular.z);
}

static void resetOdometryCallback(const void* reqin, void* resin) {
  std_srvs__srv__Trigger_Response* res =
      (std_srvs__srv__Trigger_Response*)resin;
  dc.resetOdom();
  res->success = true;
}

static void resetBoardCallback(const void* reqin, void* resin) {
  std_srvs__srv__Trigger_Response* res =
      (std_srvs__srv__Trigger_Response*)resin;
  reset_request = true;
  rosidl_runtime_c__String__assign(&res->message,
                                   "Requested board software reset");
  res->success = true;
}

static void getFirmwareVersionCallback(const void* reqin, void* resin) {
  std_srvs__srv__Trigger_Response* res =
      (std_srvs__srv__Trigger_Response*)resin;
  rosidl_runtime_c__String__assign(&res->message, FIRMWARE_VERSION);
  res->success = true;
}

static void getBoardTypeCallback(const void* reqin, void* resin) {
  std_srvs__srv__Trigger_Response* res =
      (std_srvs__srv__Trigger_Response*)resin;
  rosidl_runtime_c__String__assign(&res->message, "leocore");
  res->success = true;
}

static void wheelCmdPWMDutyCallback(const void* msgin, void* context) {
  const std_msgs__msg__Float32* msg = (std_msgs__msg__Float32*)msgin;
  WheelController* wheel = (WheelController*)context;
  wheel->disable();
  wheel->motor.setPWMDutyCycle(msg->data);
}

static void wheelCmdVelCallback(const void* msgin, void* context) {
  const std_msgs__msg__Float32* msg = (std_msgs__msg__Float32*)msgin;
  WheelController* wheel = (WheelController*)context;
  wheel->enable();
  wheel->setTargetVelocity(msg->data);
}

static void initROS() {
  // Node
  if (rclc_node_init_default(&node, "firmware", "", &support) != RCL_RET_OK)
    error_loop();

  // Executor
  rclc_executor_init(&executor, &support.context, 13, &allocator);

  // Publishers
  rclc_publisher_init_best_effort(
      &battery_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "firmware/battery");
  rclc_publisher_init_best_effort(
      &battery_averaged_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "firmware/battery_averaged");
  rclc_publisher_init_best_effort(
      &wheel_odom_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(leo_msgs, msg, WheelOdom),
      "firmware/wheel_odom");
  rclc_publisher_init_best_effort(
      &wheel_states_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(leo_msgs, msg, WheelStates),
      "firmware/wheel_states");
  rclc_publisher_init_best_effort(
      &imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(leo_msgs, msg, Imu),
      "firmware/imu");

  // Subscribers
  rclc_subscription_init_default(
      &twist_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel");
  rclc_executor_add_subscription(&executor, &twist_sub, &twist_msg,
                                 cmdVelCallback, ON_NEW_DATA);

#define WHEEL_INIT_ROS(NAME)                                   \
  rclc_subscription_init_default(                              \
      &NAME##_cmd_pwm_sub, &node,                              \
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),     \
      NAME##_cmd_pwm_topic);                                   \
  rclc_executor_add_subscription_with_context(                 \
      &executor, &NAME##_cmd_pwm_sub, &NAME##_cmd_pwm_msg,     \
      wheelCmdPWMDutyCallback, &dc.wheel_##NAME, ON_NEW_DATA); \
  rclc_subscription_init_default(                              \
      &NAME##_cmd_vel_sub, &node,                              \
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),     \
      NAME##_cmd_vel_topic);                                   \
  rclc_executor_add_subscription_with_context(                 \
      &executor, &NAME##_cmd_vel_sub, &NAME##_cmd_vel_msg,     \
      wheelCmdVelCallback, &dc.wheel_##NAME, ON_NEW_DATA);

  WHEEL_INIT_ROS(FL)
  WHEEL_INIT_ROS(RL)
  WHEEL_INIT_ROS(FR)
  WHEEL_INIT_ROS(RR)

  // Services
  rclc_service_init_default(&reset_odometry_srv, &node,
                            ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
                            "firmware/reset_odometry");
  rclc_executor_add_service(&executor, &reset_odometry_srv, &reset_odometry_req,
                            &reset_odometry_res, resetOdometryCallback);

  rclc_service_init_default(&firmware_version_srv, &node,
                            ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
                            "firmware/get_firmware_version");
  rclc_executor_add_service(&executor, &firmware_version_srv,
                            &firmware_version_req, &firmware_version_res,
                            getFirmwareVersionCallback);

  rclc_service_init_default(&board_type_srv, &node,
                            ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
                            "firmware/get_board_type");
  rclc_executor_add_service(&executor, &board_type_srv, &board_type_req,
                            &board_type_res, getBoardTypeCallback);

  rclc_service_init_default(&reset_board_srv, &node,
                            ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
                            "firmware/reset_board");
  rclc_executor_add_service(&executor, &reset_board_srv, &reset_board_req,
                            &reset_board_res, resetBoardCallback);
}

void setup() {
  rmw_uros_set_uart_transport(&huart1);

  allocator = rcl_get_default_allocator();

  // Try to connect with Micro-ROS agent
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK)
    error_loop();

  initROS();

  rmw_uros_sync_session(1000);

  imu_receiver.init();

  // Initialize Diff Drive Controller
  dc.init(params);

  configured = true;
}

void loop() {
  if (rclc_executor_spin_some(&executor, 0) != RCL_RET_OK) error_loop();

  if (publish_battery) {
    rcl_publish(&battery_pub, &battery, NULL);
    rcl_publish(&battery_averaged_pub, &battery_averaged, NULL);
    publish_battery = false;
  }

  if (publish_wheel_odom) {
    rcl_publish(&wheel_odom_pub, &wheel_odom, NULL);
    publish_wheel_odom = false;
  }

  if (publish_wheel_states) {
    rcl_publish(&wheel_states_pub, &wheel_states, NULL);
    publish_wheel_states = false;
  }

  if (publish_imu) {
    rcl_publish(&imu_pub, &imu, NULL);
    publish_imu = false;
  }
}

static builtin_interfaces__msg__Time now() {
  const int64_t nanos = rmw_uros_epoch_nanos();
  builtin_interfaces__msg__Time stamp;
  stamp.sec = nanos / (1000 * 1000 * 1000);
  stamp.nanosec = nanos % (1000 * 1000 * 1000);
  return stamp;
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

  dc.update(UPDATE_PERIOD);

  if (cnt % BATTERY_PUB_PERIOD == 0 && !publish_battery) {
    battery.data = battery_new;
    battery_averaged.data = battery_avg;

    publish_battery = true;
  }

  if (cnt % JOINTS_PUB_PERIOD == 0 && !publish_wheel_states) {
    auto dd_wheel_states = dc.getWheelStates();

    wheel_states.stamp = now();
    for (size_t i = 0; i < 4; i++) {
      wheel_states.position[i] = dd_wheel_states.position[i];
      wheel_states.velocity[i] = dd_wheel_states.velocity[i];
      wheel_states.torque[i] = dd_wheel_states.torque[i];
      wheel_states.pwm_duty_cycle[i] = dd_wheel_states.pwm_duty_cycle[i];
    }

    publish_wheel_states = true;
  }

  if (cnt % ODOM_PUB_PERIOD == 0 && !publish_wheel_odom) {
    auto dd_odom = dc.getOdom();

    wheel_odom.stamp = now();
    wheel_odom.velocity_lin = dd_odom.velocity_lin;
    wheel_odom.velocity_ang = dd_odom.velocity_ang;
    wheel_odom.pose_x = dd_odom.pose_x;
    wheel_odom.pose_y = dd_odom.pose_y;
    wheel_odom.pose_yaw = dd_odom.pose_yaw;

    publish_wheel_odom = true;
  }

  if (cnt % IMU_PUB_PERIOD == 0 && !publish_imu) {
    imu_receiver.update();

    imu.stamp = now();
    imu.temperature = imu_receiver.temp;
    imu.accel_x = imu_receiver.ax;
    imu.accel_y = imu_receiver.ay;
    imu.accel_z = imu_receiver.az;
    imu.gyro_x = imu_receiver.gx;
    imu.gyro_y = imu_receiver.gy;
    imu.gyro_z = imu_receiver.gz;

    publish_imu = true;
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
  if (huart == &ROSSERIAL_UART) {
    uart_transfer_complete_callback(huart);
  }
}