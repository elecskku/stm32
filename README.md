# 2차 발표: UART + SPI Flash (NUCLEO-F411RE + W25Q32)

PC → (UART) → STM32 → (SPI) → W25Q32 → STM32 → (UART) → PC 로 3 MB 이상 BMP를 저장/복원하고 무결성을 확인하는 코드입니다.

| 파일 | 위치 | 역할 |
|---|---|---|
| `w25q32.h / .c` | `Core/Inc`, `Core/Src` | W25Q32 SPI 드라이버 (ID, Read, Page Program, Erase) |
| `app.h / .c` | `Core/Inc`, `Core/Src` | UART 전송 프로토콜 (청크 + CRC32 + ACK/NAK 재전송, 쓰기 후 read-back 검증) |
| `flash_tool.py` | PC | 파일 전송/수신/검증, 테스트 BMP 생성, 발표용 `demo` |

---

## 1. 하드웨어 준비

### 납땜
W25Q32 모듈에 6핀 헤더를 **반드시 납땜**하세요. 꽂아만 두면 접촉 불량으로 SPI가 랜덤하게 깨집니다. (모듈이 2개니 하나는 예비)

### 배선 (수-암 점퍼선 6개: 수 → Nucleo Arduino 헤더, 암 → 모듈 핀)

| W25Q32 모듈 | NUCLEO-F411RE | STM32 핀 | 기능 |
|---|---|---|---|
| VCC | **3V3** (CN6) | – | 전원 **(5V 금지! 칩은 2.7~3.6V)** |
| GND | GND (CN6) | – | 접지 |
| CLK | D13 | PA5 | SPI1_SCK |
| DO | D12 | PA6 | SPI1_MISO |
| DI | D11 | PA7 | SPI1_MOSI |
| CS | D10 | PB6 | GPIO 출력 (FLASH_CS) |

- UART는 **배선 불필요**: USART2(PA2/PA3)가 보드의 ST-LINK USB 가상 COM 포트에 연결돼 있습니다. USB 케이블 하나로 전원 + 디버깅 + UART 모두 해결.
- PA5는 보드 LED(LD2)와 공유라서 Flash 접근 중 LED가 깜빡이는 건 정상입니다.

---

## 2. STM32CubeIDE 프로젝트 설정 (.ioc)

1. **New STM32 Project → Board Selector → NUCLEO-F411RE**, "Initialize all peripherals with their default Mode?" → **Yes**
2. **Clock Configuration**: HCLK 칸에 `100` 입력 후 Enter (APB1 = 50 MHz, APB2 = 100 MHz)
3. **Connectivity → USART2**: Mode `Asynchronous`
   - Baud Rate **921600**, 8 bit, None parity, 1 stop bit, Over Sampling 16
4. **Connectivity → SPI1**: Mode `Full-Duplex Master`, Hardware NSS `Disable`
   - Data Size 8 bits, First Bit MSB
   - Prescaler **8** (→ 12.5 Mbit/s)
   - CPOL **Low**, CPHA **1 Edge** (SPI Mode 0)
   - PA5가 LD2(GPIO)에서 SPI1_SCK로 바뀌어도 OK
5. **Pinout 화면에서 PB6** 클릭 → `GPIO_Output`, 우클릭 → Enter User Label → **`FLASH_CS`**
   - System Core → GPIO → PB6: Output level **High**, Speed **High**
6. 저장 → 코드 생성

### main.c 에 3줄 추가 (USER CODE 영역 안에)
```c
/* USER CODE BEGIN Includes */
#include "app.h"
/* USER CODE END Includes */

/* USER CODE BEGIN 2 */
App_Init();
/* USER CODE END 2 */

  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    App_Loop();
  }
  /* USER CODE END 3 */
```
`w25q32.h`, `app.h` → `Core/Inc`, `w25q32.c`, `app.c` → `Core/Src` 에 복사 후 빌드/다운로드.

---

## 3. PC 측 실행

```bash
pip install pyserial
python flash_tool.py ports                 # COM 포트 확인 (STMicroelectronics STLink Virtual COM Port)
python flash_tool.py makebmp test.bmp      # 1280x960 24bit BMP, 3,686,454 B (3.52 MB)
python flash_tool.py -p COM5 id            # SPI 동작 확인: JEDEC ID 0xEF4016 이면 OK
python flash_tool.py -p COM5 demo test.bmp # 필수 Demo 전체 실행
```

개별 명령:
```bash
python flash_tool.py -p COM5 write test.bmp      # PC → STM32 → Flash
python flash_tool.py -p COM5 read restored.bmp   # Flash → STM32 → PC
python flash_tool.py -p COM5 verify              # STM32가 Flash를 다시 읽어 CRC 재계산
python flash_tool.py -p COM5 erase               # 전체 지우기 (~10~50초)
```
직접 찍은 사진 BMP를 써도 됩니다 (최대 4,190,208 B).

### 예상 소요 시간 (921600 baud 기준)
- Erase(64 KB × 57블록): 약 5~10초
- 쓰기: 약 45~60초 / 읽기: 약 40초

---

## 4. 무결성 확인 방법 (발표 포인트)

`demo` 실행 시 다음이 모두 일치해야 **PASS**:

1. **청크 단위 CRC32** – 4 KB마다 CRC를 붙여 보내고, STM32가 틀리면 NAK → PC가 재전송
2. **쓰기 직후 Read-back 비교** – STM32가 Flash에 쓴 4 KB를 다시 읽어 `memcmp`
3. **전체 CRC32 5단계 비교** – 원본(PC) / STM32 수신 / Flash 저장값(메타데이터) / Flash에서 읽은 값 / 복원본(PC)
4. **SHA-256 + 바이트 단위 비교** – 원본 파일과 `restored.bmp`
5. 마지막으로 `restored.bmp` 를 이미지 뷰어로 열어서 보여주기

추가 시연 아이디어: 보드 USB를 뽑았다 다시 꽂은 뒤 `read` → 전원이 꺼져도 데이터가 유지됨(비휘발성)을 보여줄 수 있습니다.

---

## 5. 프로토콜 요약

| 명령 | PC → STM32 | STM32 → PC |
|---|---|---|
| `I` | `'I'` | ACK, JEDEC[3], SR1 |
| `W` | `'W'` size(u32) → 이후 청크 반복: idx(u32) len(u16) data crc32(u32) | Erase 후 ACK, 청크마다 ACK/NAK/CAN, 마지막 ACK + 전체 CRC |
| `R` | `'R'` | ACK size crc_stored data[size] crc_read |
| `V` | `'V'` | ACK size crc_stored crc_recomputed |
| `E` | `'E'` | ACK (완료 시) |

ACK=0x06, NAK=0x15, CAN=0x18(치명적 오류), 정수는 모두 little-endian, CRC32는 zlib 호환.
Flash 맵: `0x000000~0x3FEFFF` 데이터, `0x3FF000` 섹터 = 메타데이터(매직, 크기, CRC).

---

## 6. 문제 해결

| 증상 | 원인/조치 |
|---|---|
| `id` 결과 0x000000 / 0xFFFFFF | 배선(특히 DO/DI 뒤바뀜), 3V3 연결, CS=PB6(D10) 확인, 납땜 확인 |
| `id`는 되는데 쓰기 중 `CAN` | 점퍼선이 길어 신호 불량 → SPI Prescaler 16 또는 32로 낮추기 |
| SR1 경고 (block-protect) | 보호 비트 설정된 칩 → 알려주시면 해제 코드 추가 |
| 응답 없음 / timeout | COM 포트 번호, 펌웨어 다운로드 여부, 다른 터미널(PuTTY 등)이 포트를 잡고 있는지 확인 |
| retries 수가 계속 증가 | baud를 460800으로 낮추기 (CubeMX와 `-b 460800` 둘 다) |
