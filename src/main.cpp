#include <stdlib.h>
#include <string.h>

#include <cstdint>

#include "FreeRTOS.h"
#include "communication.hpp"
#include "dto/hils_rotate_command.h"
#include "enum/device_id.h"
#include "interface_communication.hpp"
#include "projdefs.h"
#include "queue.h"
#include "task.h"
#include "xgpio.h"
#include "xil_printf.h"
#include "xparameters.h"

#define TIMER_ID 1
#define DELAY_10_SECONDS 10000UL
#define DELAY_1_SECOND 1000UL
#define TIMER_CHECK_THRESHOLD 9

// ─────────────────────────────────────────
// 상수 & 타입 정의
// ─────────────────────────────────────────
namespace ServoConfig {
constexpr int SERVO_MIN_US = 500;
constexpr int SERVO_MAX_US = 2500;
constexpr int CLK_MHZ = 100;
constexpr int ANGLE_MIN = 0;
constexpr int ANGLE_MAX = 180;
constexpr int ANGLE_DEFAULT = 90;
constexpr int GPIO_CH1 = 1;  // yaw
constexpr int GPIO_CH2 = 2;  // pitch
}  // namespace ServoConfig

enum class CmdType : uint8_t {
  NULL_CMD,
  YAW_INC,
  YAW_DEC,
  PITCH_INC,
  PITCH_DEC,
  YAW_ABS,
  PITCH_ABS,
};

struct ServoCmd {
  CmdType type = CmdType::NULL_CMD;
  int value = 0;
};

static QueueHandle_t xCmdQueue = nullptr;

Network::ICommunication* i_communication = nullptr;

void command_received(const HilsRotateCommand* command) {
  ServoCmd yaw_cmd{};
  yaw_cmd.type = CmdType::YAW_ABS;
  yaw_cmd.value = command->yaw_theta / 1000;

  ServoCmd pitch_cmd{};
  pitch_cmd.type = CmdType::PITCH_ABS;
  pitch_cmd.value = command->pitch_theta / 1000;

  xQueueSend(xCmdQueue, &yaw_cmd, 0);
  xQueueSend(xCmdQueue, &pitch_cmd, 0);
}

void receive_callback(IcdId id, uint8_t* data, size_t len) {
  IcdId icd_id = static_cast<IcdId>((data[1] << 8) | data[0]);
  xil_printf("Received data with ICD ID: 0x%04X, length: %d\r\n", icd_id, len);

  switch (icd_id) {
    case IcdId::HILS_ROTATE_COMMAND:
      command_received(reinterpret_cast<const HilsRotateCommand*>(data));
      return;
    default:
      xil_printf("unimplemented icd id : 0x%04X", icd_id);
  }
}

// ─────────────────────────────────────────
// 유틸
// ─────────────────────────────────────────
static int clamp(int val, int lo, int hi) {
  if (val < lo) return lo;
  if (val > hi) return hi;
  return val;
}

static uint32_t angle_to_duty(int angle) {
  angle = clamp(angle, ServoConfig::ANGLE_MIN, ServoConfig::ANGLE_MAX);
  uint32_t us = ServoConfig::SERVO_MIN_US +
                (ServoConfig::SERVO_MAX_US - ServoConfig::SERVO_MIN_US) *
                    angle / ServoConfig::ANGLE_MAX;
  return us * ServoConfig::CLK_MHZ;
}

static void print_usage() {
  xil_printf("\r\n");
  xil_printf("┌─────────────────────────────────┐\r\n");
  xil_printf("│       Servo Control Help        │\r\n");
  xil_printf("├──────────────┬──────────────────┤\r\n");
  xil_printf("│  w           │ pitch +1도       │\r\n");
  xil_printf("│  s           │ pitch -1도       │\r\n");
  xil_printf("│  d           │ yaw   +1도       │\r\n");
  xil_printf("│  a           │ yaw   -1도       │\r\n");
  xil_printf("├──────────────┼──────────────────┤\r\n");
  xil_printf("│  l + [숫자]  │ yaw  +N도 (절대) │\r\n");
  xil_printf("│  h + [숫자]  │ yaw  -N도 (절대) │\r\n");
  xil_printf("│  k + [숫자]  │ pitch +N도 (절대)│\r\n");
  xil_printf("│  j + [숫자]  │ pitch -N도 (절대)│\r\n");
  xil_printf("└──────────────┴──────────────────┘\r\n");
  xil_printf("\r\n");
}

// ─────────────────────────────────────────
// Task 1: UART 수신 → Queue 전송
// ─────────────────────────────────────────
void uart_rx_task(void* pvParameters) {
  char pending = 0;

  print_usage();

  while (true) {
    char c = inbyte();
    outbyte(c);
    ServoCmd cmd{};
    bool enqueue = true;

    if (pending) {
      char num_buf[4] = {0};
      int idx = 0;
      char nc = c;

      while (nc >= '0' && nc <= '9' && idx < 3) {
        num_buf[idx++] = nc;
        outbyte(nc);
        nc = inbyte();
      }

      outbyte('\r');
      outbyte('\n');

      const int angle = atoi(num_buf);

      switch (pending) {
        case 'l':
          cmd = {CmdType::YAW_ABS, angle};
          break;
        case 'h':
          cmd = {CmdType::YAW_ABS, -angle};
          break;
        case 'k':
          cmd = {CmdType::PITCH_ABS, angle};
          break;
        case 'j':
          cmd = {CmdType::PITCH_ABS, -angle};
          break;
        default:
          enqueue = false;
          break;
      }
      pending = 0;

    } else {
      switch (c) {
        case 'w':
          cmd = {CmdType::PITCH_INC, 1};
          break;
        case 's':
          cmd = {CmdType::PITCH_DEC, 1};
          break;
        case 'd':
          cmd = {CmdType::YAW_INC, 1};
          break;
        case 'a':
          cmd = {CmdType::YAW_DEC, 1};
          break;
        case 'h':
        case 'j':
        case 'k':
        case 'l':
          pending = c;
          xil_printf("angle> ");
          enqueue = false;
          break;
        case '?':
          print_usage();
          enqueue = false;
          break;
        default:
          enqueue = false;
          break;
      }
    }

    if (enqueue && xCmdQueue) {
      xQueueSend(xCmdQueue, &cmd, 0);
    }
  }
}

// 상수 추가
namespace LedConfig {
constexpr int GPIO_CH = 1;
constexpr uint32_t ALL_ON = 0xF;  // LD0~LD3 전부 ON
constexpr uint32_t ALL_OFF = 0x0;
}  // namespace LedConfig

// ─────────────────────────────────────────
// Task 2: Queue 수신 → GPIO 업데이트
// ─────────────────────────────────────────
void servo_ctrl_task(void* pvParameters) {
  XGpio gpio{};
  XGpio gpio_led{};  // ← LED용

  xil_printf("init servo");
  XGpio_Initialize(&gpio, XPAR_AXI_GPIO_0_DEVICE_ID);
  XGpio_SetDataDirection(&gpio, ServoConfig::GPIO_CH1, 0);  // CH1 = yaw
  XGpio_SetDataDirection(&gpio, ServoConfig::GPIO_CH2, 0);  // CH2 = pitch

  int yaw = ServoConfig::ANGLE_DEFAULT;
  int pitch = ServoConfig::ANGLE_DEFAULT;

  XGpio_DiscreteWrite(&gpio, ServoConfig::GPIO_CH1, angle_to_duty(yaw));
  XGpio_DiscreteWrite(&gpio, ServoConfig::GPIO_CH2, angle_to_duty(pitch));

  xil_printf("[Servo] Initialized. yaw=%d pitch=%d\r\n", yaw, pitch);

  ServoCmd cmd{};

  while (true) {
    if (xQueueReceive(xCmdQueue, &cmd, portMAX_DELAY) != pdTRUE) continue;

    switch (cmd.type) {
      case CmdType::YAW_INC:
        yaw += cmd.value;
        break;
      case CmdType::YAW_DEC:
        yaw -= cmd.value;
        break;
      case CmdType::PITCH_INC:
        pitch += cmd.value;
        break;
      case CmdType::PITCH_DEC:
        pitch -= cmd.value;
        break;
      case CmdType::YAW_ABS:
        yaw = cmd.value;
        break;
      case CmdType::PITCH_ABS:
        pitch = cmd.value;
        break;
      default:
        break;
    }

    yaw = clamp(yaw, ServoConfig::ANGLE_MIN, ServoConfig::ANGLE_MAX);
    pitch = clamp(pitch, ServoConfig::ANGLE_MIN, ServoConfig::ANGLE_MAX);

    XGpio_DiscreteWrite(&gpio, ServoConfig::GPIO_CH1, angle_to_duty(yaw));
    XGpio_DiscreteWrite(&gpio, ServoConfig::GPIO_CH2, angle_to_duty(pitch));

    xil_printf("[Servo] yaw: %3d deg  |  pitch: %3d deg\r\n", yaw, pitch);
    print_usage();
  }
}

#define THREAD_STACKSIZE 1024
int main(void) {
  Network::Communication communication{};
  i_communication = &communication;
  communication.object_init(DeviceId::HILS);
  communication.register_callback(receive_callback);

  xil_printf("Hello Hils\r\n");

  xCmdQueue = xQueueCreate(64, sizeof(ServoCmd));
  configASSERT(xCmdQueue != nullptr);

  BaseType_t r1 = xTaskCreate(uart_rx_task, "UART_RX", 2048, nullptr,
                              tskIDLE_PRIORITY + 1, nullptr);
  BaseType_t r2 = xTaskCreate(servo_ctrl_task, "SERVO", 8192, nullptr,
                              tskIDLE_PRIORITY + 1, nullptr);

  xil_printf("Task create: uart=%d servo=%d\r\n", r1, r2);
  xil_printf("Starting scheduler...\r\n");

  vTaskStartScheduler();

  while (true);

  return 0;
}