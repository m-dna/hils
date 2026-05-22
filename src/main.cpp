#include <stdlib.h>
#include <string.h>

#include <atomic>
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
constexpr int ANGLE_MAX = 360;
constexpr int DUTY_MAX = 360;
constexpr int ANGLE_DEFAULT = 70;
constexpr int GPIO_CH1 = 1;      // yaw
constexpr int GPIO_CH2 = 2;      // pitch
constexpr int MAX_STEP_DEG = 2;  // 매 주기 최대 추종량 (도)
constexpr int PITCH_MIN = -60;   // IMU 기준 -60도
constexpr int PITCH_MAX = 60;    // IMU 기준 +60도 (servo ~120도)
constexpr TickType_t PERIOD_MS = pdMS_TO_TICKS(20);  // 모터 20ms 주기
}  // namespace ServoConfig
// ─────────────────────────────────────────
// 전역 목표값 — Critical Section으로 보호
// ─────────────────────────────────────────
static int g_target_yaw = ServoConfig::ANGLE_DEFAULT;
static int g_target_pitch = ServoConfig::ANGLE_DEFAULT;

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

// ─────────────────────────────────────────
// 유틸
// ─────────────────────────────────────────
static int clamp(int val, int lo, int hi) {
  if (val < lo) return lo;
  if (val > hi) return hi;
  return val;
}

/**
 * @brief 현재값(cur)을 목표값(tgt)으로 최대 step 만큼 추종
 */
static int step_toward(int cur, int tgt, int step) {
  int diff = tgt - cur;
  if (diff > step) return cur + step;
  if (diff < -step) return cur - step;
  return tgt;  // 이미 목표 도달
}

static uint32_t angle_to_duty(int angle) {
  angle = clamp(angle, ServoConfig::ANGLE_MIN, ServoConfig::ANGLE_MAX);

  float ratio = (float)angle / (float)ServoConfig::DUTY_MAX;

  float us = ServoConfig::SERVO_MIN_US +
             (ServoConfig::SERVO_MAX_US - ServoConfig::SERVO_MIN_US) * ratio;

  return (uint32_t)us * ServoConfig::CLK_MHZ;
}

// 읽기
static void get_targets(int* yaw, int* pitch) {
  taskENTER_CRITICAL();
  *yaw = g_target_yaw;
  *pitch = g_target_pitch;
  taskEXIT_CRITICAL();
}

// 쓰기
static void set_target_yaw(int val) {
  taskENTER_CRITICAL();
  g_target_yaw = clamp(val, ServoConfig::ANGLE_MIN, ServoConfig::ANGLE_MAX);
  taskEXIT_CRITICAL();
}

static void set_target_pitch(int val) {
  taskENTER_CRITICAL();
  g_target_pitch = clamp(val, ServoConfig::ANGLE_MIN, ServoConfig::ANGLE_MAX);
  taskEXIT_CRITICAL();
}

static void add_target_yaw(int delta) {
  taskENTER_CRITICAL();
  g_target_yaw = clamp(g_target_yaw + delta, ServoConfig::ANGLE_MIN,
                       ServoConfig::ANGLE_MAX);
  taskEXIT_CRITICAL();
}

static void add_target_pitch(int delta) {
  taskENTER_CRITICAL();
  g_target_pitch = clamp(g_target_pitch + delta, ServoConfig::ANGLE_MIN,
                         ServoConfig::ANGLE_MAX);
  taskEXIT_CRITICAL();
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

Network::ICommunication* i_communication = nullptr;

void command_received(const HilsRotateCommand* command) {
  add_target_yaw(command->yaw_theta / 1000);
  add_target_pitch(command->pitch_theta / 1000);
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
// Task 1: UART 수신 → 전역 목표값 갱신
// ─────────────────────────────────────────
void uart_rx_task(void* pvParameters) {
  char pending = 0;
  print_usage();

  while (true) {
    char c = inbyte();
    outbyte(c);

    if (pending) {
      // 숫자 읽기
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
      int cur_yaw, cur_pitch;
      get_targets(&cur_yaw, &cur_pitch);

      switch (pending) {
        case 'l':  // yaw 절대
          set_target_yaw(angle);
          break;
        case 'h':  // yaw 절대 (음수)
          set_target_yaw(-angle);
          break;
        case 'k':  // pitch 절대
          set_target_pitch(angle);
          break;
        case 'j':  // pitch 절대 (음수)
          set_target_pitch(-angle);
          break;
        default:
          break;
      }
      pending = 0;

    } else {
      int cur_yaw, cur_pitch;
      get_targets(&cur_yaw, &cur_pitch);

      switch (c) {
        case 'w':
          set_target_pitch(cur_pitch + 1);
          break;
        case 's':
          set_target_pitch(cur_pitch - 1);
          break;
        case 'd':
          set_target_yaw(cur_yaw + 1);
          break;
        case 'a':
          set_target_yaw(cur_yaw - 1);
          break;
        case 'h':
        case 'j':
        case 'k':
        case 'l':
          pending = c;
          xil_printf("angle> ");
          break;
        case '?':
          print_usage();
          break;
        default:
          break;
      }
    }
  }
}

// ─────────────────────────────────────────
// Task 2: 20ms 주기로 깨어나 목표값 추종
// ─────────────────────────────────────────
static XGpio gpio;
void servo_ctrl_task(void* pvParameters) {
  memset(&gpio, 0, sizeof(XGpio));

  int status = XGpio_Initialize(&gpio, XPAR_AXI_GPIO_0_DEVICE_ID);
  xil_printf("[Servo] status=%d IsReady=0x%08X\r\n", status, gpio.IsReady);
  XGpio_SetDataDirection(&gpio, ServoConfig::GPIO_CH1, 0);
  XGpio_SetDataDirection(&gpio, ServoConfig::GPIO_CH2, 0);

  int yaw = ServoConfig::ANGLE_DEFAULT;
  int pitch = ServoConfig::ANGLE_DEFAULT;

  // 초기 목표값도 현재값으로 맞춤
  int tgt_yaw, tgt_pitch;
  get_targets(&tgt_yaw, &tgt_pitch);

  XGpio_DiscreteWrite(&gpio, ServoConfig::GPIO_CH1, angle_to_duty(yaw));
  XGpio_DiscreteWrite(&gpio, ServoConfig::GPIO_CH2, angle_to_duty(pitch));
  xil_printf("[Servo] Initialized. yaw=%d pitch=%d\r\n", yaw,
             pitch - ServoConfig::ANGLE_DEFAULT);

  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (true) {
    // ── 정확히 20ms마다 깨어남 ──────────────────
    vTaskDelayUntil(&xLastWakeTime, ServoConfig::PERIOD_MS);

    // ── 목표값 읽기 ────────────────────────────
    int tgt_yaw, tgt_pitch;
    get_targets(&tgt_yaw, &tgt_pitch);

    // ── 최대 2도씩 추종 ────────────────────────
    const int new_yaw = step_toward(yaw, tgt_yaw, ServoConfig::MAX_STEP_DEG);
    const int new_pitch =
        step_toward(pitch, tgt_pitch, ServoConfig::MAX_STEP_DEG);

    // ── 값이 바뀐 경우에만 GPIO 출력 ──────────
    if (new_yaw != yaw || new_pitch != pitch) {
      yaw = new_yaw;
      pitch = new_pitch;

      XGpio_DiscreteWrite(&gpio, ServoConfig::GPIO_CH1, angle_to_duty(yaw));
      XGpio_DiscreteWrite(&gpio, ServoConfig::GPIO_CH2, angle_to_duty(pitch));

      xil_printf(
          "[Servo] yaw: %3d->%3d  pitch: %3d->%3d\r\n", yaw, tgt_yaw,
          pitch - ServoConfig::ANGLE_DEFAULT,       // 현재 pitch (IMU 기준)
          tgt_pitch - ServoConfig::ANGLE_DEFAULT);  // 목표 pitch (IMU 기준)
    }
  }
}

#define THREAD_STACKSIZE 1024
int main(void) {
  Network::Communication communication{};
  i_communication = &communication;
  communication.object_init(DeviceId::HILS);
  communication.register_callback(receive_callback);

  xil_printf("Hello Hils\r\n");

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
