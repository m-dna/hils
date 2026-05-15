// #include "FreeRTOS.h"
// #include "check_function_service.hpp"
// #include "enum/icd_id.h"
// #include "queue.h"
// #include "timers.h"

// char responseBuffer[256];
// QueueHandle_t check_function_task_queue =
//     xQueueCreate(10, sizeof(responseBuffer));

// void check_function_task(void* pvParameters) {
//   // 초기화
//   CheckFunctionService service;
//   CheckFunctionServiceState status;
//   CommMessage_t message;

//   while (true) {
//     // 1. Service 로직 업데이트 (비즈니스 로직 수행)
//     status = service.update();

//     // 2. Service가 응답 대기가 필요한 상태인지 체크
//     if (status == CheckFunctionServiceState::WAITING_FOR_RESPONSE) {
//       // [핵심] Queue에서 응답이 올 때까지 Task를 Block(잠듦) 시킴
//       // 통신 ISR에서 xQueueSend를 호출하는 순간 이 Task가 깨어남
//       if (xQueueReceive(check_function_task_queue, &message,
//                         pdMS_TO_TICKS(1000)) == pdPASS) {

//         // 데이터 처리
//         service.wakeup(message);

//         // 깨어난 직후 바로 다음 로직을 실행하기 위해 continue (Delay 건너뜀)
//         continue;
//       } else {
//         // 에러 처리
//       }
//     }

//     // WAITING 상태가 아닐 때만 주기적인 간격을 유지함
//     vTaskDelay(pdMS_TO_TICKS(10));
//   }
// }