#include <atomic>

#include "microros_allocator.h"
#include "microros_serial.h"

#include "rcl/rcl.h"
#include "rclc/executor.h"
#include "rclc/rclc.h"
#include "rclc_parameter/rclc_parameter.h"
#include "rmw_microros/rmw_microros.h"
#include "rosidl_runtime_c/string_functions.h"

#include "geometry_msgs/msg/twist.h"
#include "leo_msgs/msg/imu.h"
#include "leo_msgs/msg/wheel_odom.h"
#include "leo_msgs/msg/wheel_odom_mecanum.h"
#include "leo_msgs/msg/wheel_states.h"
#include "std_msgs/msg/empty.h"
#include "std_msgs/msg/float32.h"
#include "std_srvs/srv/trigger.h"

#include "diff_drive_lib/diff_drive_controller.hpp"
#include "diff_drive_lib/mecanum_controller.hpp"
#include "diff_drive_lib/wheel_controller.hpp"

#include "adc.h"
#include "tim.h"

#include "app/configuration.hpp"
#include "app/imu_receiver.hpp"
#include "app/microros_allocators.hpp"
#include "app/parameters.hpp"

static rcl_allocator_t allocator = rcutils_get_zero_initialized_allocator();
static rcl_init_options_t init_options;
static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;
static rclc_parameter_server_t param_server;
static rcl_timer_t ping_timer, sync_timer;

static std_msgs__msg__Float32 battery;
static std_msgs__msg__Float32 battery_averaged;
static rcl_publisher_t battery_pub;
static rcl_publisher_t battery_averaged_pub;
static diff_drive_lib::CircularBuffer<float, BATTERY_BUFFER_SIZE>
    battery_buffer;
static std::atomic_bool publish_battery(false);

static leo_msgs__msg__WheelOdom wheel_odom;
static rcl_publisher_t wheel_odom_pub;
static leo_msgs__msg__WheelOdomMecanum wheel_odom_mecanum;
static rcl_publisher_t wheel_odom_mecanum_pub;
static std::atomic_bool publish_wheel_odom(false);

static leo_msgs__msg__WheelStates wheel_states;
static rcl_publisher_t wheel_states_pub;
static std::atomic_bool publish_wheel_states(false);

static leo_msgs__msg__Imu imu;
static rcl_publisher_t imu_pub;
static std::atomic_bool publish_imu(false);

static rcl_subscription_t twist_sub;
static geometry_msgs__msg__Twist twist_msg;

static std_msgs__msg__Empty param_trigger;
static rcl_publisher_t param_trigger_pub;
static std::atomic_bool publish_param_trigger(true);

static bool mecanum_wheels = false;
static std::atomic_bool controller_initialized(false);

static size_t reset_pointer_position;

#define WHEEL_WRAPPER(NAME)                         \
  constexpr const char* NAME##_cmd_pwm_topic =      \
      "~/wheel_" #NAME "/cmd_pwm_duty";             \
  constexpr const char* NAME##_cmd_vel_topic =      \
      "~/wheel_" #NAME "/cmd_velocity";             \
  static rcl_subscription_t NAME##_cmd_pwm_sub;     \
  static rcl_subscription_t NAME##_cmd_vel_sub;     \
  static std_msgs__msg__Float32 NAME##_cmd_pwm_msg; \
  static std_msgs__msg__Float32 NAME##_cmd_vel_msg;

WHEEL_WRAPPER(FL)
WHEEL_WRAPPER(RL)
WHEEL_WRAPPER(FR)
WHEEL_WRAPPER(RR)

static rcl_service_t reset_odometry_srv, firmware_version_srv, board_type_srv,
    reset_board_srv, boot_firmware_srv;
static std_srvs__srv__Trigger_Request reset_odometry_req, firmware_version_req,
    board_type_req, reset_board_req, boot_firmware_req;
static std_srvs__srv__Trigger_Response reset_odometry_res, firmware_version_res,
    board_type_res, reset_board_res, boot_firmware_res;

static std::atomic_bool reset_request(false);
static std::atomic_bool boot_request(false);

enum class AgentStatus {
  BOOT,
  CONNECTING_TO_AGENT,
  AGENT_CONNECTED,
  AGENT_LOST
};
static AgentStatus status = AgentStatus::CONNECTING_TO_AGENT;

enum class BatteryLedStatus {
  LOW_BATTERY,
  NOT_CONNECTED,
  CONNECTED,
  BOOT,
};

static BatteryLedStatus battery_led_status = BatteryLedStatus::NOT_CONNECTED;

MotorController MotA(MOT_A_CONFIG);
MotorController MotB(MOT_B_CONFIG);
MotorController MotC(MOT_C_CONFIG);
MotorController MotD(MOT_D_CONFIG);

static uint8_t controller_buffer[std::max(
    sizeof(diff_drive_lib::DiffDriveController<VELOCITY_ROLLING_WINDOW_SIZE>),
    sizeof(diff_drive_lib::MecanumController<VELOCITY_ROLLING_WINDOW_SIZE>))];
static diff_drive_lib::RobotController<VELOCITY_ROLLING_WINDOW_SIZE>*
    controller;

enum class WheelID { FL, RL, FR, RR };
static WheelID wheel_FL = WheelID::FL;
static WheelID wheel_RL = WheelID::RL;
static WheelID wheel_FR = WheelID::FR;
static WheelID wheel_RR = WheelID::RR;

static ImuReceiver imu_receiver(&IMU_I2C);

static Parameters params;
static std::atomic_bool reload_parameters(false);

static void cmdVelCallback(const void* msgin) {
  const geometry_msgs__msg__Twist* msg =
      (const geometry_msgs__msg__Twist*)msgin;
  if (controller_initialized)
    controller->setSpeed(msg->linear.x, msg->linear.y, msg->angular.z);
}

static void resetOdometryCallback(const void* /*reqin*/, void* resin) {
  std_srvs__srv__Trigger_Response* res =
      (std_srvs__srv__Trigger_Response*)resin;
  if (controller_initialized) {
    controller->resetOdom();
    res->success = true;
  }
}

static void resetBoardCallback(const void* /*reqin*/, void* resin) {
  std_srvs__srv__Trigger_Response* res =
      (std_srvs__srv__Trigger_Response*)resin;
  reset_request = true;
  rosidl_runtime_c__String__assign(&res->message,
                                   "Requested board software reset");
  res->success = true;
}

static void getFirmwareVersionCallback(const void* /*reqin*/, void* resin) {
  std_srvs__srv__Trigger_Response* res =
      (std_srvs__srv__Trigger_Response*)resin;
  rosidl_runtime_c__String__assign(&res->message, FIRMWARE_VERSION);
  res->success = true;
}

static void getBoardTypeCallback(const void* /*reqin*/, void* resin) {
  std_srvs__srv__Trigger_Response* res =
      (std_srvs__srv__Trigger_Response*)resin;
  rosidl_runtime_c__String__assign(&res->message, "leocore");
  res->success = true;
}

static void wheelCmdPWMDutyCallback(const void* msgin, void* context) {
  const std_msgs__msg__Float32* msg = (std_msgs__msg__Float32*)msgin;
  auto wheel_id = *static_cast<const WheelID*>(context);
  diff_drive_lib::WheelController<VELOCITY_ROLLING_WINDOW_SIZE>* wheel =
      nullptr;

  if (controller_initialized) {
    switch (wheel_id) {
      case WheelID::FL:
        wheel = &controller->wheel_FL;
        break;
      case WheelID::FR:
        wheel = &controller->wheel_FR;
        break;
      case WheelID::RL:
        wheel = &controller->wheel_RL;
        break;
      case WheelID::RR:
        wheel = &controller->wheel_RR;
        break;
      default:
        break;
    }

    if (wheel) {
      wheel->disable();
      wheel->motor.setPWMDutyCycle(msg->data);
    }
  }
}

static void wheelCmdVelCallback(const void* msgin, void* context) {
  const std_msgs__msg__Float32* msg = (std_msgs__msg__Float32*)msgin;
  auto wheel_id = *static_cast<const WheelID*>(context);
  diff_drive_lib::WheelController<VELOCITY_ROLLING_WINDOW_SIZE>* wheel =
      nullptr;

  if (controller_initialized) {
    switch (wheel_id) {
      case WheelID::FL:
        wheel = &controller->wheel_FL;
        break;
      case WheelID::FR:
        wheel = &controller->wheel_FR;
        break;
      case WheelID::RL:
        wheel = &controller->wheel_RL;
        break;
      case WheelID::RR:
        wheel = &controller->wheel_RR;
        break;
      default:
        break;
    }

    if (wheel) {
      wheel->enable();
      wheel->setTargetVelocity(msg->data);
    }
  }
}

static void bootFirmwareCallback(const void* /*reqin*/, void* resin) {
  std_srvs__srv__Trigger_Response* res =
      (std_srvs__srv__Trigger_Response*)resin;
  boot_request = true;
  rosidl_runtime_c__String__assign(&res->message, "Requested firmware boot.");
  res->success = true;
}

static bool parameterChangedCallback(const Parameter*, const Parameter*,
                                     void*) {
  reload_parameters = true;
  return true;
}

static void pingTimerCallback(rcl_timer_t* /*timer*/,
                              int64_t /*last_call_time*/) {
  if (rmw_uros_ping_agent(200, 3) != RMW_RET_OK)
    status = AgentStatus::AGENT_LOST;
}

static void syncTimerCallback(rcl_timer_t* /*timer*/,
                              int64_t /*last_call_time*/) {
  rmw_uros_sync_session(1000);
}

static void initMsgs() {
  std_msgs__msg__Float32__init(&battery);
  std_msgs__msg__Float32__init(&battery_averaged);
  leo_msgs__msg__WheelOdom__init(&wheel_odom);
  leo_msgs__msg__WheelOdomMecanum__init(&wheel_odom_mecanum);
  leo_msgs__msg__WheelStates__init(&wheel_states);
  leo_msgs__msg__Imu__init(&imu);
  std_msgs__msg__Empty__init(&param_trigger);
}

#define RCCHECK(fn) \
  if ((fn != RCL_RET_OK)) return false;

static bool initROS() {
  // Init options
  init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator))
  RCCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID))

  // Support
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options,
                                         &allocator))

  // Node
  RCCHECK(rclc_node_init_default(&node, ROS_NODE_NAME, ROS_NAMESPACE, &support))

  // Executor
  RCCHECK(rclc_executor_init(&executor, &support.context,
                             16 + RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES,
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
      &wheel_states_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(leo_msgs, msg, WheelStates),
      "~/wheel_states"))
  RCCHECK(rclc_publisher_init_best_effort(
      &imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(leo_msgs, msg, Imu),
      "~/imu"))
  RCCHECK(rclc_publisher_init_best_effort(
      &param_trigger_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty), "~/param_trigger"))

  // Subscriptions
  RCCHECK(rclc_subscription_init_default(
      &twist_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel"))
  RCCHECK(rclc_executor_add_subscription(&executor, &twist_sub, &twist_msg,
                                         cmdVelCallback, ON_NEW_DATA))

#define WHEEL_INIT_ROS(NAME, ID)                           \
  RCCHECK(rclc_subscription_init_default(                  \
      &NAME##_cmd_pwm_sub, &node,                          \
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), \
      NAME##_cmd_pwm_topic))                               \
  RCCHECK(rclc_executor_add_subscription_with_context(     \
      &executor, &NAME##_cmd_pwm_sub, &NAME##_cmd_pwm_msg, \
      wheelCmdPWMDutyCallback, &ID, ON_NEW_DATA))          \
  RCCHECK(rclc_subscription_init_default(                  \
      &NAME##_cmd_vel_sub, &node,                          \
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), \
      NAME##_cmd_vel_topic))                               \
  RCCHECK(rclc_executor_add_subscription_with_context(     \
      &executor, &NAME##_cmd_vel_sub, &NAME##_cmd_vel_msg, \
      wheelCmdVelCallback, &ID, ON_NEW_DATA))

  WHEEL_INIT_ROS(FL, wheel_FL)
  WHEEL_INIT_ROS(RL, wheel_RL)
  WHEEL_INIT_ROS(FR, wheel_FR)
  WHEEL_INIT_ROS(RR, wheel_RR)

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
  RCCHECK(rclc_service_init_default(
      &boot_firmware_srv, &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger), "~/boot"))
  RCCHECK(rclc_executor_add_service(&executor, &boot_firmware_srv,
                                    &boot_firmware_req, &boot_firmware_res,
                                    &bootFirmwareCallback))

  // Parameter Server
  static rclc_parameter_options_t param_options;
  param_options.max_params = 13;
  param_options.notify_changed_over_dds = true;
  RCCHECK(rclc_parameter_server_init_with_option(&param_server, &node,
                                                 &param_options))
  if (!params.init(&param_server)) return false;
  RCCHECK(rclc_executor_add_parameter_server(&executor, &param_server,
                                             parameterChangedCallback))

  // Timers
  RCCHECK(rclc_timer_init_default(&ping_timer, &support, RCL_MS_TO_NS(5000),
                                  pingTimerCallback))
  RCCHECK(rclc_executor_add_timer(&executor, &ping_timer))
  RCCHECK(rclc_timer_init_default(&sync_timer, &support, RCL_MS_TO_NS(60000),
                                  syncTimerCallback))
  RCCHECK(rclc_executor_add_timer(&executor, &sync_timer))

  // Allocate memory
  RCCHECK(rclc_executor_prepare(&executor))

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
  (void)!rcl_service_fini(&boot_firmware_srv, &node);
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
  (void)!rcl_publisher_fini(&wheel_odom_mecanum_pub, &node);
  (void)!rcl_publisher_fini(&battery_averaged_pub, &node);
  (void)!rcl_publisher_fini(&battery_pub, &node);
  (void)!rcl_publisher_fini(&param_trigger_pub, &node);
  (void)!rcl_node_fini(&node);
  (void)!rcl_init_options_fini(&init_options);
  rclc_support_fini(&support);

  microros_heap_reset_state();
}

volatile uint16_t adc_buff[5];

static uint8_t uart_rbuffer[2048];
static uint8_t uart_tbuffer[2048];

static microros_serial_dma_stream_t stream = {
    .uart = &UROS_UART,
    .rbuffer_size = 2048,
    .rbuffer = uart_rbuffer,
    .tbuffer_size = 2048,
    .tbuffer = uart_tbuffer,
};

void setup() {
  allocator.allocate = microros_allocate;
  allocator.deallocate = microros_deallocate;
  allocator.reallocate = microros_reallocate;
  allocator.zero_allocate = microros_zero_allocate;

  (void)!rcutils_set_default_allocator(&allocator);

  microros_set_serial_transport(&stream);

  initMsgs();

  imu_receiver.init();

  status = AgentStatus::CONNECTING_TO_AGENT;
  battery_led_status = BatteryLedStatus::NOT_CONNECTED;
}

void initController() {
  mecanum_wheels = params.mecanum_wheels;
  reset_pointer_position = microros_heap_get_current_pointer();
  if (mecanum_wheels) {
    rclc_publisher_init_best_effort(
        &wheel_odom_mecanum_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(leo_msgs, msg, WheelOdomMecanum),
        "~/wheel_odom_mecanum");
    controller = new (controller_buffer)
        diff_drive_lib::MecanumController<VELOCITY_ROLLING_WINDOW_SIZE>(
            ROBOT_CONFIG);
  } else {
    rclc_publisher_init_best_effort(
        &wheel_odom_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(leo_msgs, msg, WheelOdom), "~/wheel_odom");
    controller = new (controller_buffer)
        diff_drive_lib::DiffDriveController<VELOCITY_ROLLING_WINDOW_SIZE>(
            ROBOT_CONFIG);
  }
  controller->init(params);
  controller_initialized = true;
}

void finiController() {
  controller_initialized = false;
  if (mecanum_wheels) {
    (void)!rcl_publisher_fini(&wheel_odom_mecanum_pub, &node);
  } else {
    (void)!rcl_publisher_fini(&wheel_odom_pub, &node);
  }
  controller->~RobotController();
  microros_heap_set_current_pointer(reset_pointer_position);
}

void loop() {
  static uint32_t boot_enter_time;
  switch (status) {
    case AgentStatus::CONNECTING_TO_AGENT:
      // Try to connect to uros agent
      if (rmw_uros_ping_agent(1000, 1) == RMW_RET_OK) {
        if (initROS()) {
          (void)!rcl_timer_call(&sync_timer);
          boot_enter_time = time();
          status = AgentStatus::BOOT;
        } else
          finiROS();
      }
      break;
    case AgentStatus::BOOT:
      rclc_executor_spin_some(&executor, 0);

      if (reload_parameters.exchange(false)) {
        params.update(&param_server);
      }

      if (publish_param_trigger) {
        (void)!rcl_publish(&param_trigger_pub, &param_trigger, NULL);
        publish_param_trigger = false;
      } else if (boot_request || time() - boot_enter_time >= BOOT_TIMEOUT) {
        (void)!rcl_publisher_fini(&param_trigger_pub, &node);
        // this uncomented breaks whole ROS communication
        // (void)!rclc_executor_remove_service(&executor, &boot_firmware_srv);
        // (void)!rcl_service_fini(&boot_firmware_srv, &node);
        initController();
        status = AgentStatus::AGENT_CONNECTED;
      }
      break;
    case AgentStatus::AGENT_CONNECTED:
      rclc_executor_spin_some(&executor, 0);

      if (reset_request) reset();

      if (publish_battery) {
        (void)!rcl_publish(&battery_pub, &battery, NULL);
        (void)!rcl_publish(&battery_averaged_pub, &battery_averaged, NULL);
        publish_battery = false;
      }

      if (publish_wheel_odom) {
        if (params.mecanum_wheels) {
          (void)!rcl_publish(&wheel_odom_mecanum_pub, &wheel_odom_mecanum,
                             NULL);
        } else {
          (void)!rcl_publish(&wheel_odom_pub, &wheel_odom, NULL);
        }
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

      if (reload_parameters.exchange(false)) {
        params.update(&param_server);
        if (params.mecanum_wheels != mecanum_wheels) {
          finiController();
          initController();
        } else {
          controller->updateParams(params);
        }
      }
      break;
    case AgentStatus::AGENT_LOST:
      if (controller_initialized) {
        controller->disable();
        finiController();
      }
      finiROS();
      status = AgentStatus::CONNECTING_TO_AGENT;
      break;
    default:
      break;
  }
}

static builtin_interfaces__msg__Time now() {
  const int64_t nanos = rmw_uros_epoch_nanos();
  builtin_interfaces__msg__Time stamp;
  stamp.sec = nanos / (1000 * 1000 * 1000);
  stamp.nanosec = nanos % (1000 * 1000 * 1000);
  return stamp;
}

void update_battery_led(uint32_t cnt) {
  static bool blinking = false;
  static uint8_t blinks_cnt = 0;

  switch (battery_led_status) {
    case BatteryLedStatus::LOW_BATTERY:
      if (cnt % 10 == 0) gpio_toggle(LED);
      break;
    case BatteryLedStatus::NOT_CONNECTED:
      if (cnt % 50 == 0) gpio_toggle(LED);
      break;
    case BatteryLedStatus::CONNECTED:
      gpio_reset(LED);
      break;
    case BatteryLedStatus::BOOT:
      if (blinking) {
        if (cnt % 10 == 0) {
          gpio_toggle(LED);
          ++blinks_cnt;
        }
        if (blinks_cnt >= 4) {
          blinking = false;
          blinks_cnt = 0;
        }
      } else {
        gpio_reset(LED);
        if (cnt % 100 == 0) blinking = true;
      }
      break;
    default:
      break;
  }
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
        battery_sum / static_cast<float>(std::min(BATTERY_BUFFER_SIZE,
                                                  cnt / BATTERY_PROBE_PERIOD));
  }

  if (battery_avg < params.battery_min_voltage) {
    battery_led_status = BatteryLedStatus::LOW_BATTERY;
  } else {
    if (status == AgentStatus::BOOT) {
      battery_led_status = BatteryLedStatus::BOOT;
    } else if (status != AgentStatus::AGENT_CONNECTED) {
      battery_led_status = BatteryLedStatus::NOT_CONNECTED;
    } else {
      battery_led_status = BatteryLedStatus::CONNECTED;
    }
  }

  update_battery_led(cnt);

  if (status == AgentStatus::BOOT) {
    if (cnt % PARAM_TRIGGER_PUB_PERIOD == 0 && !publish_param_trigger) {
      publish_param_trigger = true;
    }
  }

  if (status != AgentStatus::AGENT_CONNECTED || !controller_initialized) return;

  controller->update(UPDATE_PERIOD);

  if (cnt % BATTERY_PUB_PERIOD == 0 && !publish_battery) {
    battery.data = static_cast<float>(BATTERY_ADC) * BATTERY_ADC_TO_VOLTAGE;
    battery_averaged.data = battery_avg;

    publish_battery = true;
  }

  if (cnt % JOINTS_PUB_PERIOD == 0 && !publish_wheel_states) {
    auto robot_wheel_states = controller->getWheelStates();

    wheel_states.stamp = now();
    for (size_t i = 0; i < 4; i++) {
      wheel_states.position[i] = robot_wheel_states.position[i];
      wheel_states.velocity[i] = robot_wheel_states.velocity[i];
      wheel_states.torque[i] = robot_wheel_states.torque[i];
      wheel_states.pwm_duty_cycle[i] = robot_wheel_states.pwm_duty_cycle[i];
    }

    publish_wheel_states = true;
  }

  if (cnt % ODOM_PUB_PERIOD == 0 && !publish_wheel_odom) {
    auto robot_odom = controller->getOdom();

    if (params.mecanum_wheels) {
      wheel_odom_mecanum.stamp = now();
      wheel_odom_mecanum.velocity_lin_x = robot_odom.velocity_lin_x;
      wheel_odom_mecanum.velocity_lin_y = robot_odom.velocity_lin_y;
      wheel_odom_mecanum.velocity_ang = robot_odom.velocity_ang;
      wheel_odom_mecanum.pose_x = robot_odom.pose_x;
      wheel_odom_mecanum.pose_y = robot_odom.pose_y;
      wheel_odom_mecanum.pose_yaw = robot_odom.pose_yaw;
    } else {
      wheel_odom.stamp = now();
      wheel_odom.velocity_lin = robot_odom.velocity_lin_x;
      wheel_odom.velocity_ang = robot_odom.velocity_ang;
      wheel_odom.pose_x = robot_odom.pose_x;
      wheel_odom.pose_y = robot_odom.pose_y;
      wheel_odom.pose_yaw = robot_odom.pose_yaw;
    }
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

extern "C" void app_main() {
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

  HAL_TIM_Base_Start_IT(&htim11);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buff, 5);

  setup();

  while (1) {
    loop();
  }
}

extern "C" {

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
  if (htim->Instance == TIM11) {
    update();
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
  if (huart == &UROS_UART) {
    microros_uart_transfer_complete_callback(&stream);
  }
}

void microros_allocator_error(const char* msg) {
  (void)!msg;
}

void microros_allocator_fail(const char* msg) {
  (void)!msg;
  __disable_irq();
  while (1) {
  }
}
}