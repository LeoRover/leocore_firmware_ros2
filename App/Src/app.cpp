#include <atomic>

#include "microros_allocator.h"
#include "microros_serial.h"

#include "rcl/rcl.h"
#include "rclc/executor.h"
#include "rclc/rclc.h"
#include "rclc_parameter/rclc_parameter.h"
#include "rmw_microros/rmw_microros.h"
#include "rosidl_runtime_c/string_functions.h"

#include "leo_msgs/msg/imu.h"
#include "std_msgs/msg/empty.h"
#include "std_msgs/msg/float32.h"
#include "std_srvs/srv/trigger.h"

#include "adc.h"
#include "tim.h"

#include "app/configuration.hpp"
#include "app/imu_receiver.hpp"
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

static leo_msgs__msg__Imu imu;
static rcl_publisher_t imu_pub;
static std::atomic_bool publish_imu(false);

static std_msgs__msg__Empty param_trigger;
static rcl_publisher_t param_trigger_pub;
static std::atomic_bool publish_param_trigger(true);

static int leo_hardware_version = 2;

static rcl_service_t firmware_version_srv, board_type_srv,
    reset_board_srv, boot_firmware_srv;
static std_srvs__srv__Trigger_Request firmware_version_req,
    board_type_req, reset_board_req, boot_firmware_req;
static std_srvs__srv__Trigger_Response firmware_version_res,
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

static ImuReceiver imu_receiver(&IMU_I2C);

static Parameters params;
static std::atomic_bool reload_parameters(false);

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
      &imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(leo_msgs, msg, Imu),
      "~/imu"))
  RCCHECK(rclc_publisher_init_best_effort(
      &param_trigger_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty), "~/param_trigger"))

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
  param_options.max_params = 14;
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
  (void)!rcl_service_fini(&boot_firmware_srv, &node);
  (void)!rcl_publisher_fini(&imu_pub, &node);
  (void)!rcl_publisher_fini(&battery_averaged_pub, &node);
  (void)!rcl_publisher_fini(&battery_pub, &node);
  (void)!rcl_publisher_fini(&param_trigger_pub, &node);
  (void)!rcl_node_fini(&node);
  (void)!rcl_init_options_fini(&init_options);
  rclc_support_fini(&support);

  microros_heap_reset_state();
}

volatile uint16_t adc_buff[6];

static uint8_t uart_rbuffer[UROS_RBUFFER_SIZE];
static uint8_t uart_tbuffer[UROS_TBUFFER_SIZE];

static microros_serial_dma_stream_t stream = {
    .uart = &UROS_UART,
    .rbuffer_size = UROS_RBUFFER_SIZE,
    .rbuffer = uart_rbuffer,
    .tbuffer_size = UROS_TBUFFER_SIZE,
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
  leo_hardware_version = params.leo_hardware_version;

  if (leo_hardware_version == 1) {
    imu_receiver.setOrientation(ImuReceiver::Orientation::X_LEFT_Z_DOWN);
  } else {
    imu_receiver.setOrientation(ImuReceiver::Orientation::X_LEFT_Z_FORWARD);
  }
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
        // this causes hard fault later in rcl_wait
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

      if (publish_imu) {
        (void)!rcl_publish(&imu_pub, &imu, NULL);
        publish_imu = false;
      }

      if (reload_parameters.exchange(false)) {
        params.update(&param_server);
      }
      break;
    case AgentStatus::AGENT_LOST:
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

  if (status != AgentStatus::AGENT_CONNECTED) return;

  if (cnt % BATTERY_PUB_PERIOD == 0 && !publish_battery) {
    battery.data = static_cast<float>(BATTERY_ADC) * BATTERY_ADC_TO_VOLTAGE;
    battery_averaged.data = battery_avg;

    publish_battery = true;
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
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buff, 6);

  // Wait for initial ADC conversions
  delay(5);

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