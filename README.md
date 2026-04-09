# HILS

하드웨어 시뮬레이션(Hardware in the Loops) 장치 코드

## 프로젝트 구조

``` python
my_project/
├── .clang-format          # 코드 스타일 자동 정리 규칙
├── .gitattributes         # 대용량 파일 처리 및 줄바꿈(LF/CRLF) 설정
├── .gitignore             # 빌드 결과물(Debug/, Release/) 및 로컬 설정 제외
├── .gitmodules            # comm, common 등 외부 저장소 연결 정보
├── compile-commands.json  # clangd vitis 프로젝트 인식용(git에 올리지 않음)
├── app/                   # 고수준 비즈니스 로직
│   ├── inc/               # 서비스 인터페이스 (.hpp)
│   └── src/               # 서비스 로직 구현 (.cpp)
├── comm/                  # 통신 레이어 (Submodule)
│   ├── inc/               
│   └── src/               
├── common/                # 전역 및 유틸리티 레이어
│   ├── inc/               
│   └── src/               
├── hal/                   # 하드웨어 추상화 레이어
│   ├── inc/               # Peripheral 제어 인터페이스 (GPIO, PWM 등)
│   └── src/               # Vitis BSP를 호출하는 실제 드라이버 코드
├── rtos/                  # FreeRTOS 래퍼 및 관리
│   ├── inc/               # Task 핸들러, Queue 정의 헤더
│   └── src/               # Task 생성 및 스케줄링 로직
├── src/                   # 엔트리 포인트
│   └── main.c             # 하드웨어 초기화 후 RTOS 커널 시작(vTaskStartScheduler)
├── test/                  # 검증 코드
│   ├── unit/              # logic 단위 테스트 (PC 환경)
│   └── integration/       # HW 연동 통합 테스트
└── hw/                    # Vivado XSA 및 비트스트림 관리
    ├── design.xsa         # Vivado 추출 파일
    ├── constraints.xdc    # Constrants 파일
    └── backup.tcl         # 프로젝트 재생성을 위한 tcl 스크립트 등
```
