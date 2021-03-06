#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rclc_parameter/rclc_parameter.h>
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

static rcl_allocator_t allocator = rcl_get_default_allocator();
static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;
static rclc_parameter_server_t param_server;
static rcl_timer_t ping_timer, sync_timer;
static bool uros_agent_connected = false;
static bool ros_initialized = false;

static std_msgs__msg__Float32 battery;
static std_msgs__msg__Float32 battery_averaged;
static rcl_publisher_t battery_pub;
static rcl_publisher_t battery_averaged_pub;
static float battery_buffer_memory[BATTERY_BUFFER_SIZE];
static CircularBuffer<float> battery_buffer(BATTERY_BUFFER_SIZE,
                                            battery_buffer_memory);
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

#define WHEEL_WRAPPER(NAME)                                                   \
  static const char* NAME##_cmd_pwm_topic = "~/wheel_" #NAME "/cmd_pwm_duty"; \
  static const char* NAME##_cmd_vel_topic = "~/wheel_" #NAME "/cmd_velocity"; \
  static rcl_subscription_t NAME##_cmd_pwm_sub;                               \
  static rcl_subscription_t NAME##_cmd_vel_sub;                               \
  static std_msgs__msg__Float32 NAME##_cmd_pwm_msg;                           \
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

static bool parameterChangedCallback(const Parameter*, const Parameter*,
                                     void*) {
  params.update(&param_server);
  return true;
}

static void pingTimerCallback(rcl_timer_t* timer, int64_t last_call_time) {
  if (rmw_uros_ping_agent(200, 3) != RMW_RET_OK) uros_agent_connected = false;
}

static void syncTimerCallback(rcl_timer_t* timer, int64_t last_call_time) {
  rmw_uros_sync_session(1000);
}

static void initMsgs() {
  std_msgs__msg__Float32__init(&battery);
  std_msgs__msg__Float32__init(&battery_averaged);
  leo_msgs__msg__WheelOdom__init(&wheel_odom);
  leo_msgs__msg__WheelStates__init(&wheel_states);
  leo_msgs__msg__Imu__init(&imu);
}

#define RCCHECK(fn) \
  if ((fn != RCL_RET_OK)) return false;

static bool initROS() {
  // Support
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator))

  // Node
  RCCHECK(rclc_node_init_default(&node, "firmware", "", &support))

  // Executor
  RCCHECK(rclc_executor_init(&executor, &support.context,
                             15 + RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES,
                             &allocator))

  // Publishers
  RCCHECK(rclc_publisher_init_best_effort(
      &battery_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "~/battery"))
  RCCHECK(rclc_publisher_init_best_effort(
      &battery_averaged_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "~/battery_averaged"))
  RCCHECK(rclc_publisher_init_best_effort(
      &wheel_odom_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(leo_msgs, msg, WheelOdom), "~/wheel_odom"))
  RCCHECK(rclc_publisher_init_best_effort(
      &wheel_states_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(leo_msgs, msg, WheelStates),
      "~/wheel_states"))
  RCCHECK(rclc_publisher_init_best_effort(
      &imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(leo_msgs, msg, Imu),
      "~/imu"))

  // Subscriptions
  RCCHECK(rclc_subscription_init_default(
      &twist_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel"))
  RCCHECK(rclc_executor_add_subscription(&executor, &twist_sub, &twist_msg,
                                         cmdVelCallback, ON_NEW_DATA))

#define WHEEL_INIT_ROS(NAME)                                   \
  RCCHECK(rclc_subscription_init_default(                      \
      &NAME##_cmd_pwm_sub, &node,                              \
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),     \
      NAME##_cmd_pwm_topic))                                   \
  RCCHECK(rclc_executor_add_subscription_with_context(         \
      &executor, &NAME##_cmd_pwm_sub, &NAME##_cmd_pwm_msg,     \
      wheelCmdPWMDutyCallback, &dc.wheel_##NAME, ON_NEW_DATA)) \
  RCCHECK(rclc_subscription_init_default(                      \
      &NAME##_cmd_vel_sub, &node,                              \
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),     \
      NAME##_cmd_vel_topic))                                   \
  RCCHECK(rclc_executor_add_subscription_with_context(         \
      &executor, &NAME##_cmd_vel_sub, &NAME##_cmd_vel_msg,     \
      wheelCmdVelCallback, &dc.wheel_##NAME, ON_NEW_DATA))

  WHEEL_INIT_ROS(FL)
  WHEEL_INIT_ROS(RL)
  WHEEL_INIT_ROS(FR)
  WHEEL_INIT_ROS(RR)

  // Services
  RCCHECK(rclc_service_init_default(
      &reset_odometry_srv, &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger), "~/reset_odometry"))
  RCCHECK(rclc_executor_add_service(&executor, &reset_odometry_srv,
                                    &reset_odometry_req, &reset_odometry_res,
                                    resetOdometryCallback))
  RCCHECK(rclc_service_init_default(
      &firmware_version_srv, &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
      "~/get_firmware_version"))
  RCCHECK(rclc_executor_add_service(
      &executor, &firmware_version_srv, &firmware_version_req,
      &firmware_version_res, getFirmwareVersionCallback))
  RCCHECK(rclc_service_init_default(
      &board_type_srv, &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger), "~/get_board_type"))
  RCCHECK(rclc_executor_add_service(&executor, &board_type_srv, &board_type_req,
                                    &board_type_res, getBoardTypeCallback))
  RCCHECK(rclc_service_init_default(
      &reset_board_srv, &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger), "~/reset_board"))
  RCCHECK(rclc_executor_add_service(&executor, &reset_board_srv,
                                    &reset_board_req, &reset_board_res,
                                    resetBoardCallback))

  // Parameter Server
  static rclc_parameter_options_t param_options;
  param_options.max_params = 11;
  param_options.notify_changed_over_dds = true;
  RCCHECK(rclc_parameter_server_init_with_option(&param_server, &node,
                                                 &param_options))
  if (!params.init(&param_server)) return false;
  RCCHECK(rclc_executor_add_parameter_server(&executor, &param_server,
                                             parameterChangedCallback))

  RCCHECK(rclc_executor_prepare(&executor))

  // Synchronize clock
  rmw_uros_sync_session(1000);

  RCCHECK(rclc_timer_init_default(&ping_timer, &support, RCL_MS_TO_NS(5000),
                                  pingTimerCallback))
  RCCHECK(rclc_executor_add_timer(&executor, &ping_timer))
  RCCHECK(rclc_timer_init_default(&sync_timer, &support, RCL_MS_TO_NS(60000),
                                  syncTimerCallback))
  RCCHECK(rclc_executor_add_timer(&executor, &sync_timer))

  return true;
}

static void finiROS() {
  rclc_executor_fini(&executor);
  rclc_parameter_server_fini(&param_server, &node);
  (void)!rcl_timer_fini(&ping_timer);
  (void)!rcl_timer_fini(&sync_timer);
  (void)!rcl_service_fini(&reset_board_srv, &node);
  (void)!rcl_service_fini(&board_type_srv, &node);
  (void)!rcl_service_fini(&firmware_version_srv, &node);
  (void)!rcl_service_fini(&reset_odometry_srv, &node);
  (void)!rcl_subscription_fini(&twist_sub, &node);
  (void)!rcl_subscription_fini(&FL_cmd_vel_sub, &node);
  (void)!rcl_subscription_fini(&RL_cmd_vel_sub, &node);
  (void)!rcl_subscription_fini(&FR_cmd_vel_sub, &node);
  (void)!rcl_subscription_fini(&RR_cmd_vel_sub, &node);
  (void)!rcl_subscription_fini(&FL_cmd_pwm_sub, &node);
  (void)!rcl_subscription_fini(&RL_cmd_pwm_sub, &node);
  (void)!rcl_subscription_fini(&FR_cmd_pwm_sub, &node);
  (void)!rcl_subscription_fini(&RR_cmd_pwm_sub, &node);
  (void)!rcl_publisher_fini(&imu_pub, &node);
  (void)!rcl_publisher_fini(&wheel_states_pub, &node);
  (void)!rcl_publisher_fini(&wheel_odom_pub, &node);
  (void)!rcl_publisher_fini(&battery_averaged_pub, &node);
  (void)!rcl_publisher_fini(&battery_pub, &node);
  (void)!rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  rmw_uros_set_uart_transport(&UROS_UART);

  initMsgs();

  imu_receiver.init();

  // Initialize Diff Drive Controller
  dc.init(params);
}

void loop() {
  // Try to connect to uros agent
  if (!ros_initialized && rmw_uros_ping_agent(1000, 1) == RMW_RET_OK) {
    ros_initialized = initROS();
    if (ros_initialized) {
      uros_agent_connected = true;
      (void)!rcl_timer_call(&sync_timer);
    } else
      finiROS();
  }

  if (!ros_initialized) return;

  // Handle agent disconnect
  if (!uros_agent_connected) {
    finiROS();
    ros_initialized = false;
    return;
  }

  rclc_executor_spin_some(&executor, 0);

  if (publish_battery) {
    (void)!rcl_publish(&battery_pub, &battery, NULL);
    (void)!rcl_publish(&battery_averaged_pub, &battery_averaged, NULL);
    publish_battery = false;
  }

  if (publish_wheel_odom) {
    (void)!rcl_publish(&wheel_odom_pub, &wheel_odom, NULL);
    publish_wheel_odom = false;
  }

  if (publish_wheel_states) {
    (void)!rcl_publish(&wheel_states_pub, &wheel_states, NULL);
    publish_wheel_states = false;
  }

  if (publish_imu) {
    (void)!rcl_publish(&imu_pub, &imu, NULL);
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

  if (cnt % BATTERY_PROBE_PERIOD == 0) {
    battery_sum += battery_new;
    battery_sum -= battery_buffer.push_back(battery_new);
    battery_avg =
        battery_sum / static_cast<float>(std::min(BATTERY_BUFFER_SIZE, cnt));
  }

  if (battery_avg < params.battery_min_voltage) {
    if (cnt % 10 == 0) gpio_toggle(LED);
  } else {
    if (!ros_initialized) {
      if (cnt % 50 == 0) gpio_toggle(LED);
    } else {
      gpio_reset(LED);
    }
  }

  if (!ros_initialized) return;

  if (reset_request) reset();

  dc.update(UPDATE_PERIOD);

  if (cnt % BATTERY_PUB_PERIOD == 0 && !publish_battery) {
    battery.data = static_cast<float>(BATTERY_ADC) * BATTERY_ADC_TO_VOLTAGE;
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
  if (huart == &UROS_UART) {
    uart_transfer_complete_callback(huart);
  }
}