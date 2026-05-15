#include "dto/pin_control_req.h"
#include "dto/position_res.h"
#include "dto/target_distance_res.h"
#include "enum/icd_id.h"
#include "interface_communication.hpp"

enum class CheckFunctionServiceState {
  IDLE = 0,  // 대기 상태 (UI 명령 대기)

  // [준비 단계] - Preparation Phase
  PREP_SKR_SENDING,  // SKR 기능점검 명령 전송 중
  PREP_INS_SENDING,  // INS 기능점검 명령 전송 중
  PREP_ACT_SENDING,  // ACT 기능점검 명령 전송 중

  // [루프 단계] - Loop Execution Phase (N회 반복)
  LOOP_GET_SKR_POS,  // SKR 목표위치 요청 및 대기
  LOOP_GET_INS_ATT,  // INS 자세 데이터 요청 및 대기
  LOOP_ACT_CONTROL,  // 알고리즘 계산 및 ACT 제어 요청

  // [종료 단계] - Termination Phase
  TERM_SKR_SENDING,  // SKR 종료 명령 전송 중
  TERM_INS_SENDING,  // INS 종료 명령 전송 중
  TERM_ACT_SENDING,  // ACT 종료 명령 전송 중

  // [특수 상태]
  WAITING_FOR_RESPONSE,  // (Task에게 알리는 용도) 현재 응답 대기 중
  COMPLETE,              // 모든 시퀀스 정상 종료
  ERROR_STATE            // 타임아웃 또는 장비 이상 발생
};

// 이건 통신계층에 만들거임
typedef struct {
  IcdId icd_id;
  void* p_payload;
  size_t size;
} CommMessage_t;

class CheckFunctionService {
 private:
  Network::ICommunication* communication;
  CheckFunctionServiceState currentState = CheckFunctionServiceState::IDLE;
  TargetDistanceRes distanceRes;
  PositionRes positionRes;
  PinControlReq pinControlReq;

  int loopCount = 0;

  // 단계별로 해야할 함수 구현 필요(cpp)
  void idle();
  void prepSkrSending();
  void prepInsSending();
  void prepActSending();
  void loopGetSkrPos();
  void loopGetInsAtt();
  void loopActControl();
  void termSkrSending();
  void termInsSending();
  void termActSending();
  void waitingForResponse();
  void complete();

 public:
  // 깨어났을때 할 일
  void wakeup(CommMessage_t message);

  // Task가 주기적으로 호출하는 함수 -> 작업 후 상태 반환
  CheckFunctionServiceState update() {
    switch (currentState) {
      case CheckFunctionServiceState::IDLE:
        idle();
        return currentState;
      case CheckFunctionServiceState::PREP_SKR_SENDING:
        prepSkrSending();
        return currentState;
      case CheckFunctionServiceState::PREP_INS_SENDING:
        prepInsSending();
        return currentState;
      case CheckFunctionServiceState::PREP_ACT_SENDING:
        prepActSending();
        return currentState;
      case CheckFunctionServiceState::LOOP_GET_SKR_POS:
        loopGetSkrPos();
        return currentState;
      case CheckFunctionServiceState::LOOP_GET_INS_ATT:
        loopGetInsAtt();
        return currentState;
      case CheckFunctionServiceState::LOOP_ACT_CONTROL:
        loopActControl();
        return currentState;
      case CheckFunctionServiceState::TERM_SKR_SENDING:
        termSkrSending();
        return currentState;
      case CheckFunctionServiceState::TERM_INS_SENDING:
        termInsSending();
        return currentState;
      case CheckFunctionServiceState::TERM_ACT_SENDING:
        termActSending();
        return currentState;
      case CheckFunctionServiceState::WAITING_FOR_RESPONSE:
        waitingForResponse();
        return currentState;
      case CheckFunctionServiceState::COMPLETE:
        complete();
        return currentState;
    }
  }
};