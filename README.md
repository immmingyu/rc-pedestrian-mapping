# RC Pedestrian Mapping

RC카에 단 카메라로 보행자를 탐지하고, 탐지한 위치를 GPS 좌표로 바꿔 위성지도 위에 실시간으로 표시하는 프로젝트입니다.
3인 팀 프로젝트의 통합 저장소로, Raspberry Pi 4에서 실행하는 C++ 프로그램이 들어 있습니다.

- 보행자 탐지 모델의 학습·실험 과정은 별도 저장소에 정리했습니다: **[rc-pedestrian-detection](https://github.com/immmingyu/rc-pedestrian-detection)**

---

## 역할

| 구분 | 담당 |
|---|---|
| 제공받은 것 | 탐지 모델 학습 코드(`detection/train_person_detector.py`)는 프로젝트를 지도해주신 선배님이 제공한 예시 코드를 바탕으로 수정한 것 |
| 팀원 1 | 하드웨어 제어(I2C·PWM·서보·모터·GPIO), 센서(IMU, GPS 수신), 카메라·키보드 입력, 메인 프로그램(`main.cpp`) |
| 팀원 2 | GPS 좌표 → 위성지도 픽셀 변환, 보행자 위치 계산, 위성지도 투영·경로 표시, GPS 수신 모듈 공동 작업 |
| 나 (immmingyu) | 보행자 탐지 모델 학습, ONNX Runtime 기반 탐지 모듈(`detection/`) 통합, CMake·CI의 ONNX Runtime 빌드 구성 |

---

## 시스템 구조

```
 카메라 (640x480)
   │  frame
   ▼
 보행자 탐지  ─────────────── detection/PersonDetector  (ONNX Runtime, 별도 스레드)
   │  바운딩 박스 → 발 위치(박스 하단 중앙)
   ▼
 위치 계산  ──────────────── coord/PersonLocationCalculator
   │  ① 픽셀 → 거리·상대각   (카메라 화각·높이·기울기)
   │  ② 상대각 + IMU 방위각 → 절대 방위각
   │  ③ RC카 GPS 좌표 + 거리·방위각 → 보행자 위도·경도
   ▼
 위성지도 표시  ──────────── coord/SeoilCoordController
      위도·경도 → 지도 픽셀 (네 모서리 기준 호모그래피)
      RC카 위치·이동 경로·보행자 위치를 sat_img.png 위에 그림

 GPS (별도 스레드) ── sensors/GpsController ── RC카 위도·경도 → coord/RcCarDataManager
 IMU (메인 루프)   ── sensors/ImuController ── RC카 방위각(heading) → coord/RcCarDataManager
```

지도 표시: 초록 점은 RC카 현재 위치, 노란 선은 최근 이동 경로(최대 100개 지점), 빨간 점은 탐지된 보행자입니다.

> 참고: 거리 계산(`PersonLocationCalculator.cpp`)에는 실험으로 정한 보정 계수(`VISUAL_WEIGHT = 8.0`)가 곱해져 있습니다. 측량 수준의 정확한 위치가 아니라 지도 위에 대략적인 위치를 보여주는 용도입니다.

### 폴더

| 경로 | 역할 |
|---|---|
| `main.cpp` | 장치 초기화, GPS·AI 스레드 실행, 카메라/지도 화면 출력, 키보드 조작 |
| `hardware/` | I2C 통신, PWM 보드, 서보(카메라 팬·틸트, 조향), DC 모터, GPIO |
| `sensors/` | GPS 수신(TCP로 NMEA RMC 문장 파싱), BNO055 IMU 방위각 |
| `coord/` | RC카 상태 관리, 보행자 위경도 계산, 위경도 → 위성지도 픽셀 변환 및 그리기 |
| `detection/` | ONNX 탐지 모듈(`PersonDetector`), 모델 파일, 학습 스크립트(`train_person_detector.py`) |
| `utils/` | 카메라 캡처(libcamera GStreamer, 실패 시 V4L2), 터미널 키 입력 |
| `sat_img.png` | 실험 장소 위성지도 이미지 |
| `.github/workflows/ci.yml` | PR 시 Ubuntu에서 빌드 확인 |

<!-- TODO: 지도 출처 -->

---

## 탐지 모델

`main.cpp`는 **실험 10 모델**을 불러옵니다.

- 파일: `detection/models/person_detector_script_10_stride16.onnx`
- 입력 320×240, 사람 1개 클래스, grid 기반 커스텀 탐지기
- 설정값: confidence 0.25, NMS IoU 0.3

실험 11 모델(`person_detector_script_11_lite.onnx`)은 더 가벼워서 RPi4에서 13~15 FPS가 나왔지만, 검출 품질이 부족해 최종 시연에는 실험 10 모델을 사용했습니다. 실험 11 모델 파일도 비교용으로 저장소에 남겨 두었습니다.

모델 구조, 학습 데이터, 실험 기록은 [rc-pedestrian-detection](https://github.com/immmingyu/rc-pedestrian-detection)을 참고하세요.

---

## 실행 방법

### 필요한 하드웨어

코드 기준으로 다음 장치가 모두 연결되어 있어야 프로그램이 시작됩니다.

| 장치 | 코드에서 사용하는 방식 |
|---|---|
| Raspberry Pi 4 | 실행 환경 (Linux) |
| 카메라 | libcamera(GStreamer `libcamerasrc`)로 먼저 열고, 실패하면 V4L2 장치 0번 사용 |
| PWM 제어 보드 (Robot HAT) | I2C `/dev/i2c-1`, 주소 `0x14` — 서보 채널 0(카메라 팬), 1(카메라 틸트), 2(조향) 및 모터 속도 |
| DC 모터 2개 | 방향 제어 GPIO 23, 24 (`/dev/gpiochip0`) |
| IMU (BNO055) | I2C `/dev/i2c-1`, 주소 `0x28` — 방위각 |
| GPS | iPhone의 GPS2IP 앱이 TCP로 보내는 NMEA(`$GPRMC`/`$GNRMC`)를 수신. 주소는 `main.cpp`에 `172.20.10.1:11123`(iPhone 개인용 핫스팟)으로 고정되어 있어, 환경이 다르면 수정해야 합니다. |
| 모니터 | 카메라 화면과 위성지도를 OpenCV 창으로 띄웁니다. |

> **하드웨어 없이는 메인 프로그램이 실행되지 않습니다.** 시작하자마자 I2C 장치, GPS TCP 연결, IMU, 카메라를 순서대로 열고, 하나라도 실패하면 오류를 내고 종료합니다.
> 하드웨어 없이 할 수 있는 것은 **빌드**까지입니다(CI도 하드웨어 없이 빌드만 확인합니다).

또한 위성지도와 좌표 변환은 `sat_img.png`가 담고 있는 실험 장소에 맞춰져 있습니다. 다른 장소에서 쓰려면 `sat_img.png`와 `coord/SeoilCoordController.h`의 네 모서리 위경도를 함께 바꿔야 합니다.

### 의존성

- Linux (Raspberry Pi OS 등). Linux 전용 헤더(`linux/i2c-dev.h` 등)를 사용하므로 Windows/macOS에서는 빌드되지 않습니다.
- CMake 3.14 이상, C++17 컴파일러
- OpenCV (`libopencv-dev`) — 카메라를 libcamera로 열려면 OpenCV의 GStreamer 지원과 libcamera GStreamer 플러그인이 필요합니다. 없으면 V4L2로 대신 엽니다.
- ONNX Runtime 1.20.1 C/C++ SDK — 공식 CMake 패키지가 없어 압축을 푼 경로를 직접 지정해야 합니다.

### 빌드

```bash
sudo apt-get install -y build-essential cmake libopencv-dev

# ONNX Runtime (RPi4: aarch64 / 일반 PC: x64)
curl -LO https://github.com/microsoft/onnxruntime/releases/download/v1.20.1/onnxruntime-linux-aarch64-1.20.1.tgz
tar xzf onnxruntime-linux-aarch64-1.20.1.tgz
export ONNXRUNTIME_ROOT=$PWD/onnxruntime-linux-aarch64-1.20.1

cmake -B build -DONNXRUNTIME_ROOT=$ONNXRUNTIME_ROOT
cmake --build build
```

`ONNXRUNTIME_ROOT`는 `-D` 옵션 대신 환경변수로만 지정해도 됩니다. ONNX Runtime 라이브러리 경로는 실행 파일의 rpath에 들어가므로 `LD_LIBRARY_PATH`를 따로 설정할 필요가 없습니다.

### 실행

```bash
cd build
./my_app
```

**반드시 `build/` 폴더 안에서 실행해야 합니다.** 프로그램이 모델과 지도 이미지를 현재 폴더 기준 상대경로로 읽기 때문입니다.

- 모델: `../detection/models/person_detector_script_10_stride16.onnx`
- 지도: `../sat_img.png` (없으면 시작 단계에서 종료)

키 입력은 OpenCV 창이 아니라 **실행한 터미널**에서 받습니다(터미널이 아니면 시작 단계에서 종료). 화면 출력용 디스플레이가 있는 환경의 터미널에서 실행하고, 키를 누를 때는 터미널 창에 포커스를 두세요.

### 조작키

| 키 | 동작 |
|---|---|
| `W` / `S` | 전진 / 후진 (속도 40%) |
| `A` / `D` | 좌 / 우 조향 (5°씩, 최대 ±30°) |
| `I` / `K` | 카메라 위 / 아래 (5°씩, 최대 ±45°) |
| `J` / `L` | 카메라 좌 / 우 (5°씩, 최대 ±90°) |
| `R` | 카메라 정면으로 리셋 |
| `Space` | 정지 |
| `Q` | 종료 (터미널) |
| `ESC` | 종료 (OpenCV 창) |
