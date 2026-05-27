#include <math.h>
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

// ─────────────────────────────────────────
// 상수 & 타입 정의
// ─────────────────────────────────────────
namespace ServoConfig {
constexpr int SERVO_MIN_US = 1000;
constexpr int SERVO_MAX_US = 2000;
constexpr int SERVO_MID_US = 1500;
constexpr int CLK_MHZ = 50;
constexpr int GPIO_CH1 = 1;  // yaw
constexpr int GPIO_CH2 = 2;  // pitch
constexpr int MAX_STEP_US = 30;
constexpr TickType_t PERIOD_MS = pdMS_TO_TICKS(20);

// 각도 범위 (캘리브레이션 테이블 기준)
constexpr float PITCH_DEG_MIN = -40.0f;
constexpr float PITCH_DEG_MAX = 40.0f;
constexpr float YAW_DEG_MIN = -40.0f;
constexpr float YAW_DEG_MAX = 50.0f;

// wsad 1회 이동량
constexpr float STEP_DEG = 1.0f;
}  // namespace ServoConfig

// ─────────────────────────────────────────
// 캘리브레이션 테이블
// ─────────────────────────────────────────
struct CalibPoint {
  float deg;
  int us;
};

static const CalibPoint kPitchTable[] = {
    {-40.0f, 1120}, {-30.0f, 1220}, {-20.0f, 1312},
    {-10.0f, 1409}, {0.0f, 1500},   {10.0f, 1605},
    {20.0f, 1705},  {30.0f, 1792},  {40.0f, 1890},
};
static const CalibPoint kYawTable[] = {
    {-40.0f, 1100}, {-30.0f, 1200}, {-20.0f, 1300}, {-10.0f, 1395},
    {0.0f, 1500},   {10.0f, 1600},  {20.0f, 1695},  {30.0f, 1795},
    {40.0f, 1875},  {50.0f, 1975},
};
constexpr int kPitchTableSize = sizeof(kPitchTable) / sizeof(kPitchTable[0]);
constexpr int kYawTableSize = sizeof(kYawTable) / sizeof(kYawTable[0]);

// ─────────────────────────────────────────
// 선형 보간 (0.5도 해상도로 클램프 후 보간)
// ─────────────────────────────────────────
static float snap_half(float deg) {
  // 0.5도 단위로 반올림
  return roundf(deg * 2.0f) / 2.0f;
}

static int deg_to_us(float deg, const CalibPoint* table, int size) {
  deg = snap_half(deg);

  // 범위 클램프
  if (deg <= table[0].deg) return table[0].us;
  if (deg >= table[size - 1].deg) return table[size - 1].us;

  // 구간 탐색
  for (int i = 0; i < size - 1; ++i) {
    if (deg >= table[i].deg && deg <= table[i + 1].deg) {
      float t = (deg - table[i].deg) / (table[i + 1].deg - table[i].deg);
      return (int)roundf(table[i].us + t * (table[i + 1].us - table[i].us));
    }
  }
  return table[size - 1].us;  // 안전망
}

static int pitch_deg_to_us(float deg) {
  return deg_to_us(deg, kPitchTable, kPitchTableSize);
}
static int yaw_deg_to_us(float deg) {
  return deg_to_us(deg, kYawTable, kYawTableSize);
}

// ─────────────────────────────────────────
// 전역 목표값 — μs 단위 + 각도 상태
// ─────────────────────────────────────────
static int g_target_yaw_us = ServoConfig::SERVO_MID_US;
static int g_target_pitch_us = ServoConfig::SERVO_MID_US;
static float g_current_yaw_deg = 0.0f;  // wsad 추적용 현재 각도
static float g_current_pitch_deg = 0.0f;

// ─────────────────────────────────────────
// 유틸
// ─────────────────────────────────────────
static int clamp(int val, int lo, int hi) {
  if (val < lo) return lo;
  if (val > hi) return hi;
  return val;
}
static float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}
static int step_toward(int cur, int tgt, int step) {
  int diff = tgt - cur;
  if (diff > step) return cur + step;
  if (diff < -step) return cur - step;
  return tgt;
}
static uint32_t us_to_duty(int us) {
  us = clamp(us, ServoConfig::SERVO_MIN_US, ServoConfig::SERVO_MAX_US);
  return (uint32_t)us * ServoConfig::CLK_MHZ;
}

// ─────────────────────────────────────────
// 전역 목표값 접근자
// ─────────────────────────────────────────
static void get_targets(int* yaw_us, int* pitch_us) {
  taskENTER_CRITICAL();
  *yaw_us = g_target_yaw_us;
  *pitch_us = g_target_pitch_us;
  taskEXIT_CRITICAL();
}
static void set_target_yaw_us(int us) {
  taskENTER_CRITICAL();
  g_target_yaw_us =
      clamp(us, ServoConfig::SERVO_MIN_US, ServoConfig::SERVO_MAX_US);
  taskEXIT_CRITICAL();
}
static void set_target_pitch_us(int us) {
  taskENTER_CRITICAL();
  g_target_pitch_us =
      clamp(us, ServoConfig::SERVO_MIN_US, ServoConfig::SERVO_MAX_US);
  taskEXIT_CRITICAL();
}
static void add_target_yaw_us(int delta) {
  taskENTER_CRITICAL();
  g_target_yaw_us = clamp(g_target_yaw_us + delta, ServoConfig::SERVO_MIN_US,
                          ServoConfig::SERVO_MAX_US);
  taskEXIT_CRITICAL();
}
static void add_target_pitch_us(int delta) {
  taskENTER_CRITICAL();
  g_target_pitch_us =
      clamp(g_target_pitch_us + delta, ServoConfig::SERVO_MIN_US,
            ServoConfig::SERVO_MAX_US);
  taskEXIT_CRITICAL();
}

// 각도로 목표 설정 (보간 적용)
static void set_target_yaw_deg(float deg) {
  deg = clampf(deg, ServoConfig::YAW_DEG_MIN, ServoConfig::YAW_DEG_MAX);
  taskENTER_CRITICAL();
  g_current_yaw_deg = deg;
  g_target_yaw_us = yaw_deg_to_us(deg);
  taskEXIT_CRITICAL();
}
static void set_target_pitch_deg(float deg) {
  deg = clampf(deg, ServoConfig::PITCH_DEG_MIN, ServoConfig::PITCH_DEG_MAX);
  taskENTER_CRITICAL();
  g_current_pitch_deg = deg;
  g_target_pitch_us = pitch_deg_to_us(deg);
  taskEXIT_CRITICAL();
}

// wsad: 현재 각도에서 ±1도 이동
static void add_target_yaw_deg(float delta) {
  taskENTER_CRITICAL();
  float new_deg = clampf(g_current_yaw_deg + delta, ServoConfig::YAW_DEG_MIN,
                         ServoConfig::YAW_DEG_MAX);
  g_current_yaw_deg = new_deg;
  g_target_yaw_us = yaw_deg_to_us(new_deg);
  taskEXIT_CRITICAL();
}
static void add_target_pitch_deg(float delta) {
  taskENTER_CRITICAL();
  float new_deg =
      clampf(g_current_pitch_deg + delta, ServoConfig::PITCH_DEG_MIN,
             ServoConfig::PITCH_DEG_MAX);
  g_current_pitch_deg = new_deg;
  g_target_pitch_us = pitch_deg_to_us(new_deg);
  taskEXIT_CRITICAL();
}
static void get_current_degs(float* yaw, float* pitch) {
  taskENTER_CRITICAL();
  *yaw = g_current_yaw_deg;
  *pitch = g_current_pitch_deg;
  taskEXIT_CRITICAL();
}

// ─────────────────────────────────────────
// 도움말
// ─────────────────────────────────────────
static void print_usage() {
  xil_printf("\r\n");
  xil_printf("┌─────────────────────────────────────────────────┐\r\n");
  xil_printf("│            Servo Control Help                   │\r\n");
  xil_printf("├──────────────────────┬──────────────────────────┤\r\n");
  xil_printf("│ [μs 직접 입력]        │                          │\r\n");
  xil_printf("│  y<값> (예: y1500)   │ yaw   μs 절대값          │\r\n");
  xil_printf("│  p<값> (예: p2000)   │ pitch μs 절대값          │\r\n");
  xil_printf("│  Y<값> (예: Y+100)   │ yaw   μs 상대값          │\r\n");
  xil_printf("│  P<값> (예: P-50)    │ pitch μs 상대값          │\r\n");
  xil_printf("├──────────────────────┼──────────────────────────┤\r\n");
  xil_printf("│ [각도 입력 (0.5도 보간)] │                        │\r\n");
  xil_printf("│  A<값> (예: a-20.5)  │ yaw   각도 절대값        │\r\n");
  xil_printf("│  E<값> (예: e10.0)   │ pitch 각도 절대값        │\r\n");
  xil_printf("├──────────────────────┼──────────────────────────┤\r\n");
  xil_printf("│ [wsad: 1도씩 이동]   │                          │\r\n");
  xil_printf("│  w  pitch +1도       │  s  pitch -1도           │\r\n");
  xil_printf("│  d  yaw   +1도       │  a  yaw   -1도           │\r\n");
  xil_printf("│  (대문자 WSAD: 10도)  │                          │\r\n");
  xil_printf("├──────────────────────┼──────────────────────────┤\r\n");
  xil_printf("│  ?  도움말           │  c  현재 각도/μs 출력    │\r\n");
  xil_printf("└──────────────────────┴──────────────────────────┘\r\n");
  xil_printf("  Yaw   범위: %.1f ~ %.1f 도\r\n", ServoConfig::YAW_DEG_MIN,
             ServoConfig::YAW_DEG_MAX);
  xil_printf("  Pitch 범위: %.1f ~ %.1f 도\r\n", ServoConfig::PITCH_DEG_MIN,
             ServoConfig::PITCH_DEG_MAX);
  xil_printf("\r\n");
}

// ─────────────────────────────────────────
// 통신 콜백
// ─────────────────────────────────────────
Network::ICommunication* i_communication = nullptr;

void command_received(const HilsRotateCommand* command) {
  float yaw_deg = command->yaw_theta / 1000.0f;
  float pitch_deg = command->pitch_theta / 1000.0f;
  set_target_yaw_deg(yaw_deg);
  set_target_pitch_deg(pitch_deg);
}

void receive_callback(IcdId id, uint8_t* data, size_t len) {
  IcdId icd_id = static_cast<IcdId>((data[1] << 8) | data[0]);
  switch (icd_id) {
    case IcdId::HILS_ROTATE_COMMAND:
      command_received(reinterpret_cast<const HilsRotateCommand*>(data));
      return;
    default:
      xil_printf("unimplemented icd id: 0x%04X\r\n", icd_id);
  }
}

// ─────────────────────────────────────────
// UART 파서 헬퍼
// ─────────────────────────────────────────
static int read_signed_int() {
  char buf[8] = {0};
  int idx = 0, sign = 1;

  char c = inbyte();
  outbyte(c);
  if (c == '+') {
    c = inbyte();
    outbyte(c);
  } else if (c == '-') {
    sign = -1;
    c = inbyte();
    outbyte(c);
  }

  while (c >= '0' && c <= '9' && idx < 5) {
    buf[idx++] = c;
    c = inbyte();
    outbyte(c);
  }
  outbyte('\r');
  outbyte('\n');
  return sign * atoi(buf);
}

// 부호 있는 실수 읽기 (예: -20.5, +10, 30.0)
static float read_signed_float() {
  char buf[16] = {0};
  int idx = 0;
  float sign = 1.0f;

  char c = inbyte();
  outbyte(c);
  if (c == '+') {
    c = inbyte();
    outbyte(c);
  } else if (c == '-') {
    sign = -1.0f;
    c = inbyte();
    outbyte(c);
  }

  while (((c >= '0' && c <= '9') || c == '.') && idx < 10) {
    buf[idx++] = c;
    c = inbyte();
    outbyte(c);
  }
  outbyte('\r');
  outbyte('\n');
  return sign * strtof(buf, nullptr);
}

// ─────────────────────────────────────────
// Task 1: UART 수신
//
//  μs 직접:  y/p (절대), Y/P (상대)
//  각도:     a<deg> = yaw 절대,  e<deg> = pitch 절대
//  wsad:     w/s = pitch ±1도,  d/a = yaw ±1도 (대문자 10도)
//  기타:     ? = 도움말,  c = 현재 상태
// ─────────────────────────────────────────
void uart_rx_task(void* pvParameters) {
  print_usage();

  while (true) {
    char cmd = inbyte();
    outbyte(cmd);

    int cur_yaw_us, cur_pitch_us;
    float cur_yaw_deg, cur_pitch_deg;
    get_targets(&cur_yaw_us, &cur_pitch_us);
    get_current_degs(&cur_yaw_deg, &cur_pitch_deg);

    switch (cmd) {
      // ── μs 절대값 ──────────────────────
      case 'y': {
        int us = read_signed_int();
        set_target_yaw_us(us);
        xil_printf("[CMD] yaw   -> %d us\r\n", us);
        break;
      }
      case 'p': {
        int us = read_signed_int();
        set_target_pitch_us(us);
        xil_printf("[CMD] pitch -> %d us\r\n", us);
        break;
      }
      // ── μs 상대값 ──────────────────────
      case 'Y': {
        int delta = read_signed_int();
        add_target_yaw_us(delta);
        get_targets(&cur_yaw_us, &cur_pitch_us);
        xil_printf("[CMD] yaw   += %d us -> %d us\r\n", delta, cur_yaw_us);
        break;
      }
      case 'P': {
        int delta = read_signed_int();
        add_target_pitch_us(delta);
        get_targets(&cur_yaw_us, &cur_pitch_us);
        xil_printf("[CMD] pitch += %d us -> %d us\r\n", delta, cur_pitch_us);
        break;
      }

      // ── 각도 절대값 (0.5도 보간) ────────
      case 'A': {  // yaw 각도 절대
        float deg = read_signed_float();
        set_target_yaw_deg(deg);
        get_targets(&cur_yaw_us, &cur_pitch_us);
        get_current_degs(&cur_yaw_deg, &cur_pitch_deg);
        xil_printf("[CMD] yaw   -> %.1f deg (%d us)\r\n", cur_yaw_deg,
                   cur_yaw_us);
        break;
      }
      case 'E': {  // pitch 각도 절대
        float deg = read_signed_float();
        set_target_pitch_deg(deg);
        get_targets(&cur_yaw_us, &cur_pitch_us);
        get_current_degs(&cur_yaw_deg, &cur_pitch_deg);
        xil_printf("[CMD] pitch -> %.1f deg (%d us)\r\n", cur_pitch_deg,
                   cur_pitch_us);
        break;
      }

      // ── wsad: 1도씩 이동 ────────────────
      case 'w': {
        add_target_pitch_deg(+ServoConfig::STEP_DEG);
        get_current_degs(&cur_yaw_deg, &cur_pitch_deg);
        get_targets(&cur_yaw_us, &cur_pitch_us);
        xil_printf("[WSAD] pitch +1 -> %.1f deg (%d us)\r\n", cur_pitch_deg,
                   cur_pitch_us);
        break;
      }
      case 's': {
        add_target_pitch_deg(-ServoConfig::STEP_DEG);
        get_current_degs(&cur_yaw_deg, &cur_pitch_deg);
        get_targets(&cur_yaw_us, &cur_pitch_us);
        xil_printf("[WSAD] pitch -1 -> %.1f deg (%d us)\r\n", cur_pitch_deg,
                   cur_pitch_us);
        break;
      }
      case 'd': {
        add_target_yaw_deg(+ServoConfig::STEP_DEG);
        get_current_degs(&cur_yaw_deg, &cur_pitch_deg);
        get_targets(&cur_yaw_us, &cur_pitch_us);
        xil_printf("[WSAD] yaw   +1 -> %.1f deg (%d us)\r\n", cur_yaw_deg,
                   cur_yaw_us);
        break;
      }
      case 'a': {
        add_target_yaw_deg(-ServoConfig::STEP_DEG);
        get_current_degs(&cur_yaw_deg, &cur_pitch_deg);
        get_targets(&cur_yaw_us, &cur_pitch_us);
        xil_printf("[WSAD] yaw   -1 -> %.1f deg (%d us)\r\n", cur_yaw_deg,
                   cur_yaw_us);
        break;
      }
      // ── WSAD: 10도씩 이동 ───────────────
      case 'W': {
        add_target_pitch_deg(+10.0f);
        get_current_degs(&cur_yaw_deg, &cur_pitch_deg);
        get_targets(&cur_yaw_us, &cur_pitch_us);
        xil_printf("[WSAD] pitch +10 -> %.1f deg (%d us)\r\n", cur_pitch_deg,
                   cur_pitch_us);
        break;
      }
      case 'S': {
        add_target_pitch_deg(-10.0f);
        get_current_degs(&cur_yaw_deg, &cur_pitch_deg);
        get_targets(&cur_yaw_us, &cur_pitch_us);
        xil_printf("[WSAD] pitch -10 -> %.1f deg (%d us)\r\n", cur_pitch_deg,
                   cur_pitch_us);
        break;
      }
      case 'D': {
        add_target_yaw_deg(+10.0f);
        get_current_degs(&cur_yaw_deg, &cur_pitch_deg);
        get_targets(&cur_yaw_us, &cur_pitch_us);
        xil_printf("[WSAD] yaw   +10 -> %.1f deg (%d us)\r\n", cur_yaw_deg,
                   cur_yaw_us);
        break;
      }
      // 소문자 a 와 충돌하므로 대문자 A를 각도 절대값으로 사용
      // (위에서 이미 처리)

      // ── 현재 상태 출력 ──────────────────
      case 'c': {
        outbyte('\r');
        outbyte('\n');
        xil_printf("[STATUS] yaw:   %.1f deg  %d us\r\n", cur_yaw_deg,
                   cur_yaw_us);
        xil_printf("[STATUS] pitch: %.1f deg  %d us\r\n", cur_pitch_deg,
                   cur_pitch_us);
        break;
      }

      case '?':
        print_usage();
        break;
      default:
        break;
    }
  }
}

// ─────────────────────────────────────────
// Task 2: 20ms 주기 서보 제어 (변경 없음)
// ─────────────────────────────────────────
static XGpio gpio;

void servo_ctrl_task(void* pvParameters) {
  memset(&gpio, 0, sizeof(XGpio));

  int status = XGpio_Initialize(&gpio, XPAR_AXI_GPIO_0_DEVICE_ID);
  xil_printf("[Servo] init status=%d IsReady=0x%08X\r\n", status, gpio.IsReady);
  XGpio_SetDataDirection(&gpio, ServoConfig::GPIO_CH1, 0);
  XGpio_SetDataDirection(&gpio, ServoConfig::GPIO_CH2, 0);

  int yaw_us = ServoConfig::SERVO_MID_US;
  int pitch_us = ServoConfig::SERVO_MID_US;

  XGpio_DiscreteWrite(&gpio, ServoConfig::GPIO_CH1, us_to_duty(yaw_us));
  XGpio_DiscreteWrite(&gpio, ServoConfig::GPIO_CH2, us_to_duty(pitch_us));
  xil_printf("[Servo] Initialized. yaw=%d us  pitch=%d us\r\n", yaw_us,
             pitch_us);

  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (true) {
    vTaskDelayUntil(&xLastWakeTime, ServoConfig::PERIOD_MS);

    int tgt_yaw, tgt_pitch;
    get_targets(&tgt_yaw, &tgt_pitch);

    const int new_yaw = tgt_yaw;
    const int new_pitch = tgt_pitch;

    if (new_yaw != yaw_us || new_pitch != pitch_us) {
      yaw_us = new_yaw;
      pitch_us = new_pitch;

      XGpio_DiscreteWrite(&gpio, ServoConfig::GPIO_CH1, us_to_duty(yaw_us));
      XGpio_DiscreteWrite(&gpio, ServoConfig::GPIO_CH2, us_to_duty(pitch_us));

      xil_printf("[Servo] yaw: %4d us (tgt %4d)  pitch: %4d us (tgt %4d)\r\n",
                 yaw_us, tgt_yaw, pitch_us, tgt_pitch);
    }
  }
}

// ─────────────────────────────────────────
// main
// ─────────────────────────────────────────
int main(void) {
  Network::Communication communication{};
  i_communication = &communication;
  communication.init(DeviceId::HILS);
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