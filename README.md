# scale-car-autorace

1/10 스케일 자율주행 RC카의 2024년 자율주행 경진대회 출전 코드입니다.
최종 주행은 2024-11-22 대구에서 진행했습니다.

딥러닝 없이 **고전 컴퓨터 비전과 기하학적 LiDAR 처리, PID 조향 제어**만으로 구현했습니다.
학습 가중치나 데이터셋은 없습니다.

## Hardware

| 항목 | 값 |
|---|---|
| 플랫폼 | MIT racecar-v2 (F1TENTH 계열) |
| 구동 | VESC 브러시리스 모터 컨트롤러, `AckermannDriveStamped` |
| LiDAR | Slamtec RPLIDAR (2D, `/scan`) |
| 카메라 | USB 카메라 640×480, **어안(equidistant) 왜곡 보정** |
| IMU | Razor 9-DoF |
| 온보드 PC | Intel NUC11, ROS 1 Noetic |

## Packages

| 패키지 | 역할 |
|---|---|
| **`daecar`** | 2024년 재작업본, 주력. 진입점은 `scripts/main_20.py` |
| `scalecar` | 이전 세대. 프레임 폭 640px 기준(`middle_lane=320`)이고 `daecar` 는 320px(`middle_lane=160`). 출처 보존용으로 남깁니다 |
| `wego` | 텔레옵 + rviz 센서 뷰 |
| `limo_marker` | AR 마커 실습 8단계 (구독 → 정지 → 거리 → 각도 → ID → 추종 → 통합) |

### `daecar` 내부 구성

| 파일 | 역할 |
|---|---|
| `scripts/SlideWindow.py` | 버드아이 뷰 변환 → 히스토그램 피크 → **30윈도우 슬라이딩 차선 탐색** → polyfit → 차선 중심 |
| `scripts/HSV.py`, `HSVtest.py` | HSV 임계값 튜너. 흰 차선 `[0,0,140]`, 적색 구간 면적비(`red_area_ratio`) 검출 |
| `scripts/my_detect_obs.py` | `/scan` 각도 섹터링 → `lidar_warning`, `static_or_dynamic`, `light_obj`, `right_obj`, 차단기 검출(`blocker_status`, 3프레임 디바운스) |
| `scripts/rabacon_drive.py` | `obstacle_detector` 의 `/raw_obstacles` 원형 군집을 좌우 ±1.2m 로 분리 → 라바콘 슬라럼 조향값 발행 |
| `scripts/child_sign.py` | `/ar_pose_marker` (ar_track_alvar) 구독 → `sign_id` 발행. 마커 ID 로 차선 선택 |
| `scripts/main_20.py` | 우선순위 상태기계 + PID 조향. 정지선 → 어린이보호구역 감속 → 적색구간 감속 → 터널 → 로터리 양보 → 차단기 → 라바콘 → 정적/동적 장애물 회피 → 차선 유지. 주차 시퀀스 포함 |

측위(localization) 모듈은 없습니다. SLAM 도 odom 융합도 맵도 쓰지 않는 **순수 반응형** 구조입니다.

## Run

```bash
roslaunch daecar scale_total_launch.launch
```

`main_20.py` · `my_detect_obs.py` · `child_sign.py` · `rabacon_drive.py` 와
`racecar` 텔레옵, `obstacle_detector` 노드렛, `ar_track_alvar`, `image_proc` 를 함께 띄웁니다.

`scale_total_launch12.launch` 는 `main_20` 만 남긴 디버그용 축소판입니다.

## Vendored dependencies

벤더 ROS 패키지 10개는 이 저장소에 포함하지 않습니다 (`razor_imu_9dof` 의 GPLv3 포함).

```bash
wstool init src
wstool merge .rosinstall
wstool update
```

라이선스 현황은 `LICENSE-NOTES.md` 참조.
`.rosinstall` 항목 중 5개는 upstream 을 추정한 것이라 사용 전 확인이 필요합니다.

## Known quirks

**1. `src/CMakeLists.txt` 가 심볼릭 링크가 아닙니다.**
Windows 에서 `core.symlinks=false` 라 49바이트 텍스트 파일로 퇴화한 상태로 커밋됐습니다.
Linux 에서 클론한 뒤 복구하세요.

```bash
ln -sf /opt/ros/noetic/share/catkin/cmake/toplevel.cmake src/CMakeLists.txt
```

**2. `src/daecar/scripts/my_detect_obs 11.22 대구 최종.py`**
공백·한글·점이 섞인 파일명이지만 **의도적으로 유지**합니다 — 11월 22일 대구 최종 주행본이라는 출처 정보입니다.

**3. `package.xml` 의 maintainer 가 `wego@todo.todo` 입니다.**
교육용 키트의 `catkin_create_pkg` 템플릿 기본값이며 실제 주소가 아닙니다.

## Not in this repository

| 항목 | 크기 | 사유 |
|---|---|---|
| `build/`, `devel/` | 130 MB | catkin 빌드 산출물 |
| 벤더 패키지 10개 | 53 MB | 라이선스 분리. `.rosinstall` 참조 |
| 중첩 `.git` 5개 | 29 MB | 벤더 저장소의 자체 이력 |
| `src/calibration/*.png` | 9 MB | 체커보드 촬영 원본 34장 |
| `src/y_list*.txt` | 1.1 MB | roslaunch 콘솔 덤프. 로컬 호스트명·경로 노출 |
| `DAP_2024_autorace.zip` | 90 MB | 이 폴더 자체의 중복 아카이브 (저장소 루트 밖) |

전부 로컬에 원본 그대로 남아 있습니다.
