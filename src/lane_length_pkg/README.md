# lane_length_pkg

ROS 2 기반 차선 인식 및 차선 추종 패키지입니다.  
카메라 영상으로 좌/우 두 차선을 검출하고, 두 차선의 중점 중심선을 따라 TurtleBot3에 `/cmd_vel`을 보냅니다. LiDAR `/scan`은 중심 경로 위 전방 장애물 감속/정지 게이트로 함께 사용합니다.

---

## 노드 구성 개요

```
[카메라]                                      [LiDAR]
   │
   │                                             │ /scan
   ▼
detection_node               ← 좌/우 차선 검출, 중점 중심선 생성
   │  /lane_error_center_m
   │  /lane_error_right_m
   │  /lane_heading_error
   │  /lane_status
   │  /lane_width_m
   │  /lane_centerline_base_path
   │
   └──────────────► integration_node             ← 카메라 path + LiDAR 장애물 통합
                   │           │ /fused/centerline_path
                   │           │ /fused/obstacles
                   │           │ /fused/lane_width_m
                   │           ▼
                        direct live topics
                               │ /lane_error_center_m
                               │ /lane_error_right_m
                               │ /lane_heading_error
                               │ /lane_status
                               │ /lane_guidance_source
                               │
                    ┌──────────┴──────────┐
                    ▼                     ▼
         decision_node          control_node
         (통과 가능 판단)          (중점 path pursuit + LiDAR 게이트)
               │                         │
               │ /control_mode           │ /cmd_vel → turtlebot3_node
               │ /safe_stop              ▲
               └─────────────────────────┘
                                         │
                           /fused/centerline_path + /scan

viewer_node는 `/image_raw`, `/fused/*`, `/cmd_vel`, 상태 토픽을 구독해서 카메라 영상과 top-down 주행 상태를 표시합니다.
```

---

## 노드별 상세 설명

### 1. `detection_node`

**역할**: 카메라 이미지에서 차선을 검출하고, 로봇이 차선 중앙/우측에서 얼마나 벗어났는지(오차)를 계산합니다.

**주요 동작**:
- 입력 이미지에서 흰색/노란색 차선 픽셀을 추출 (HLS/HSV 컬러 필터링)
- BEV(Bird's Eye View) 호모그래피 변환으로 탑뷰 이미지 생성
- 슬라이딩 윈도우 알고리즘으로 좌/우 차선 위치 추적 및 2차 다항식 피팅
- 메트릭 호모그래피(`H.npy`, `Hinv.npy`)로 픽셀→미터 변환
- 검출된 좌/우 차선으로부터 중점 중심선(centerline) 생성
- 중심/우측 오차, heading error 계산
- 차선 폭(미터) 및 차선 경계 path 퍼블리시

**Subscribe**:
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/image_raw` | `sensor_msgs/Image` | 카메라 원본 이미지 |

**Publish**:
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/lane_error_center_m` | `Float32` | 차선 중앙으로부터의 횡방향 오차 (m) |
| `/lane_error_right_m` | `Float32` | 우측 차선 기준 횡방향 오차 (m) |
| `/lane_heading_error` | `Float32` | 차선 방향과의 각도 오차 (rad) |
| `/lane_status` | `String` | 차선 검출 상태 (`ok` / `lost`) |
| `/lane_width_m` | `Float32` | 현재 차선 폭 (m) |
| `/lane_centerline_base_path` | `nav_msgs/Path` | 차선 중심선 경로 (base_link 기준) |
| `/lane_left_boundary_base_path` | `nav_msgs/Path` | 좌측 차선 경계 경로 |
| `/lane_right_boundary_base_path` | `nav_msgs/Path` | 우측 차선 경계 경로 |

**주요 파라미터**:
| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `image_topic` | `/image_raw` | 입력 이미지 토픽 |
| `process_fps` | `10.0` | 이미지 처리 주기 (Hz) |
| `use_camera_rectification` | `True` | 카메라 왜곡 보정 사용 여부 |
| `camera_info_yaml` | `config/default_cam.yaml` | `/camera_info`가 없을 때 사용할 캘리브레이션 YAML |
| `rectification_model` | `auto` | `auto` / `plumb_bob` / `equidistant` / `fisheye` |
| `fisheye_balance` | `0.0` | 어안 보정 시 주변 시야 보존 정도 (0.0=더 많이 펴고 crop, 1.0=시야 보존) |
| `fisheye_fov_scale` | `1.0` | 어안 보정 후 가상 초점거리 스케일 |
| `publish_rectified_image` | `False` | 보정된 이미지를 `/image_rectified`로 퍼블리시 |
| `use_metric_homography` | `True` | 미터 단위 호모그래피 사용 여부 |
| `px_per_m` | `100.0` | BEV 이미지의 픽셀/미터 비율 |
| `nwindows` | `6` | 슬라이딩 윈도우 개수 |
| `smooth_alpha` | `0.08` | 차선 피팅 EMA 스무딩 계수 |
| `follow_mode` | `center` | 추종 모드 (`center` / `right`) |

---

### 2. `decision_node`

**역할**: 현재 차선 폭을 로봇 폭과 비교하여 통과 가능 여부를 판단하고, 제어 모드와 안전 정지 명령을 퍼블리시합니다.

**주요 동작**:
- 차선 폭을 `live_width` → `held_width` 순서로 폴백하여 결정
- `차선폭 >= 로봇폭 + margin + hysteresis` → **NORMAL_CENTER_DRIVE** (주행 허용)
- `차선폭 < 로봇폭 + margin` → **PASS_BLOCKED** (정지)
- 히스테리시스 적용으로 모드 채터링 방지
- 차선 상태 타임아웃, 차선 lost, 유효 폭 없음 시 모두 PASS_BLOCKED로 전환

**Subscribe**:
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/lane_width_m` | `Float32` | 현재 검출된 차선 폭 |
| `/lane_status` | `String` | live 차선 상태 |
| `/lane_guidance_source` | `String` | 현재 guidance 소스 |

**Publish**:
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/control_mode` | `String` | 제어 모드 (`NORMAL_CENTER_DRIVE` / `PASS_BLOCKED`) |
| `/safe_stop` | `Bool` | 안전 정지 플래그 |
| `/decision_status` | `String` | 결정 이유 로그 문자열 |

**주요 파라미터**:
| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `robot_width_m` | `0.19` | 로봇 폭 (m) |
| `width_margin_m` | `0.00` | 통과 판정 여유 폭 (m) |
| `width_hysteresis_m` | `0.01` | 히스테리시스 폭 (m) |
| `lane_timeout_sec` | `0.5` | 차선 상태 유효 시간 (s) |
| `hold_last_width_sec` | `1.0` | 마지막 유효 폭 유지 시간 (s) |
| `stop_on_lane_lost` | `True` | 차선 lost 시 정지 여부 |
| `decision_hz` | `10.0` | 판단 주기 (Hz) |

---

### 3. `control_node`

**역할**: 카메라가 만든 두 차선 중점 중심선 path를 motion_node 방식의 다점 pursuit로 추종하고, LiDAR 전방 장애물 정보를 섞어 `cmd_vel`을 계산합니다. path가 없을 때만 기존 active guidance 오차 기반 제어로 폴백합니다.

**주요 동작**:
- `safe_stop=True` 또는 `control_mode=PASS_BLOCKED`이면 즉시 정지 명령 출력
- live 카메라 상태에서는 `/lane_centerline_base_path`를 우선 사용
- 중심선의 여러 점을 가중 평균하여 pursuit/heading 조향 계산
- `/scan` 포인트가 중심 경로의 전방 corridor 안에 있으면 감속 또는 정지
- guidance source(`live` / `none`)에 따라 제어 게인과 deadband 적용
- 오차가 클수록 속도 감소 (slow_error_m 이상부터 선형 감속, hard_stop_error_m 이상이면 최저속 또는 제자리 회전)
- lookahead 보상 옵션: 카메라-baselink 오프셋 및 전방 예측 오차 보정

**Subscribe**:
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/lane_error_center_m` | `Float32` | 중앙 횡방향 오차 (m) |
| `/lane_error_right_m` | `Float32` | 우측 횡방향 오차 (m) |
| `/lane_heading_error` | `Float32` | heading 오차 (rad) |
| `/lane_status` | `String` | 차선 상태 |
| `/lane_guidance_source` | `String` | guidance 소스 |
| `/control_mode` | `String` | 제어 모드 |
| `/safe_stop` | `Bool` | 안전 정지 플래그 |
| `/fused/centerline_path` | `nav_msgs/Path` | 좌/우 차선 중점 중심 경로 |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR 전방 장애물 게이트 |

**Publish**:
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 로봇 속도 명령 |

**주요 파라미터**:
| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `control_hz` | `20.0` | 제어 주기 (Hz) |
| `kp_m` | `0.22` | 횡방향 오차 P 게인 |
| `k_heading` | `0.03` | heading 오차 게인 |
| `nominal_speed` | `0.028` | 기본 전진 속도 (m/s) |
| `min_speed` | `0.018` | 최소 전진 속도 (m/s) |
| `hard_stop_error_m` | `0.25` | 이 오차 이상이면 최저속/제자리 회전 (m) |
| `default_follow_mode` | `center` | 기본 추종 기준 (`center` / `right`) |

---

## 실행 방법

### 빌드

```bash
cd ~/fsd_ws
colcon build --packages-select lane_length_pkg
source install/setup.bash
```

### 1. TurtleBot3 로봇 실행

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
```

### 2. 카메라 노드 실행

```bash
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p pixel_format:=yuyv \
  -p camera_info_url:="file:///home/jhp/fsd_ws/src/lane_length_pkg/config/default_cam.yaml"
```

### 3. 차선 추종 파이프라인 실행

```bash
ros2 launch lane_length_pkg lane_pipeline.launch.py
```

어안 렌즈 캘리브레이션 YAML의 `distortion_model`이 `equidistant` 또는 `fisheye`이면 detection 노드가 입력 영상을 먼저 펴고 그 결과를 차선 검출/BEV 변환에 사용합니다. YAML 모델명이 맞지 않으면 강제로 지정할 수 있습니다:

```bash
ros2 run lane_length_pkg detection_node --ros-args \
  -p rectification_model:=fisheye \
  -p camera_info_yaml:=/path/to/fisheye_cam.yaml \
  -p publish_rectified_image:=true
```

뷰어 없이 실행하려면:

```bash
ros2 launch lane_length_pkg lane_pipeline.launch.py use_viewer:=false
```

개별 실행이 필요하면 새 이름을 사용합니다:

```bash
ros2 run lane_length_pkg detection_node
ros2 run lane_length_pkg integration_node
ros2 run lane_length_pkg decision_node
ros2 run lane_length_pkg control_node
ros2 run lane_length_pkg viewer_node
```

---

## 의존성

- ROS 2 (Humble 이상)
- `rclpy`, `cv_bridge`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `std_msgs`, `visualization_msgs`
- `opencv-python`, `numpy`
