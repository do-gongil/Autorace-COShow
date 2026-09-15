# 서드파티 라이선스 메모

이 저장소에는 **자작 패키지 4개**(`daecar`, `scalecar`, `wego`, `limo_marker`)만 들어 있습니다.
빌드에 필요한 벤더 ROS 패키지 10개는 `.rosinstall` 로 받아오며, 각자의 라이선스를 따릅니다.

| 패키지 | 라이선스 | 비고 |
|---|---|---|
| `razor_imu_9dof` | **GPLv3** (BSD 듀얼) | 이 저장소에 포함하지 않은 가장 큰 이유. 통째로 커밋하면 copyleft 가 전체에 번집니다. |
| `ar_track_alvar` | LGPL-2.1 | |
| `aruco_ros` | BSD | PAL Robotics |
| `fiducials` | BSD | Ubiquity Robotics |
| `lidar_tracking` | — | kostaskonkk/datmo 의 포크. 내부에 `kalman-cpp` 별도 라이선스 포함 |
| `obstacle_detector` | BSD | 저자 논문 PDF 동봉 |
| `racecar` | BSD 계열 | MIT Lincoln Laboratory |
| `rplidar_ros` | BSD | Slamtec |
| `usb_cam` | BSD | ros-drivers |
| `vesc` | BSD 계열 | MIT racecar |

자작 패키지는 대회 팀 공동 저작물이므로 저장소 자체에는 `LICENSE` 를 두지 않았습니다.
