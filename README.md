# Clothoid-R Seminar - LiDAR Detection 실습

> **Autonomous Racing Simulator** 환경에서 LiDAR 포인트클라우드를 처리하는 ROS 2 실습 패키지입니다.
> ROI 필터링 → 다운샘플링 → 지면 제거 → 클러스터링 → 바운딩 박스까지 단계별로 학습합니다.

---

## 환경

| 항목 | 버전 |
|------|------|
| OS | Ubuntu 24.04 |
| ROS 2 | Kilted |
| Gazebo | Ionic |
| 시뮬레이터 | [Autonomous-Racing-Simulator](https://github.com/ttaehyun/Autonomous-Racing-Simulator) |
| Docker Image | `rth0824/autonomous-racing-simulator:ver1.1` |

---

## 패키지 구조

```
lidar_practice_ws/
└── src/
    └── lidar_detection/
        ├── lidar_detection/
        │   └── lidar_pipeline.py   ← 메인 실습 파일
        ├── launch/
        │   └── lidar_tutorial.launch.py
        ├── package.xml
        ├── setup.py
        └── setup.cfg
```

---

## LiDAR 처리 파이프라인

```
PointCloud2 입력 (/car1/scan/points)
        │
        ▼
┌─────────────────┐
│  1. ROI 필터링   │  관심 영역 밖 포인트 제거
└────────┬────────┘
         │
         ▼
┌──────────────────────┐
│  2. Voxel 다운샘플링   │  복셀 격자로 포인트 수 축소
└────────┬─────────────┘
         │
         ▼
┌─────────────────┐
│  3. 지면 제거     │  그리드 셀 최저점 기준 바닥 제거
└────────┬────────┘
         │
         ▼
┌──────────────────────────┐
│  4. Euclidean Clustering │  KD-Tree + BFS 객체 분리
└────────┬─────────────────┘
         │
         ▼
┌─────────────────────┐
│  5. Bounding Box    │  AABB 3D 박스 생성 및 발행
└─────────────────────┘
         │
         ├── /preprocessed_points  (PointCloud2)
         └── /bounding_boxes       (MarkerArray)
```

---

## 실습 전 준비

### 1. Docker 컨테이너 실행

```bash
# 이미지 pull (최초 1회)
docker pull rth0824/autonomous-racing-simulator:ver1.1

# 컨테이너 실행 스크립트 (run_racing_sim.sh 참고)
docker run --gpus all -it --privileged \
  -e DISPLAY=$DISPLAY \
  -e __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v <볼륨경로>:/home/user/volume \
  --network host \
  --name autonomous-racing-sim \
  rth0824/autonomous-racing-simulator:ver1.1 bash
```

### 2. 시뮬레이터 실행 (터미널 3개)

```bash
# [터미널 1] Gazebo 맵 실행
gz sim -r ~/volume/Autonomous-Racing-Simulator/simulate_ws/src/server/map/racemap.sdf

# [터미널 2] 차량 스폰 및 ROS-Gazebo 브릿지 실행
source ~/volume/Autonomous-Racing-Simulator/simulate_ws/install/setup.bash
ros2 launch server spawn_car.launch.py

# [터미널 3] 키보드 조종
python3 ~/volume/Autonomous-Racing-Simulator/simulate_ws/src/server/src/key_teleop.py
```

---

## 빌드 및 실행

```bash
cd ~/volume/Autonomous-Racing-Simulator/lidar_practice_ws

# 빌드
colcon build --packages-select lidar_detection
source install/setup.bash

# 실행
ros2 launch lidar_detection lidar_tutorial.launch.py

# 차량 이름이 다를 경우
ros2 launch lidar_detection lidar_tutorial.launch.py car_name:=car2
```

---

## 파라미터 튜닝

`lidar_pipeline.py` 상단의 파라미터 블록에서 값을 바꿔가며 결과를 확인하세요.

```python
# ── 1. ROI ──────────────────────────────────────────────────
ROI_X_MIN, ROI_X_MAX =  0.0, 20.0   # 전방 거리 (m)
ROI_Y_MIN, ROI_Y_MAX = -5.0,  5.0   # 좌우 너비 (m)
ROI_Z_MIN, ROI_Z_MAX = -2.0,  1.5   # 높이 범위 (m)

# ── 2. Voxel Downsampling ────────────────────────────────────
VOXEL_SIZE = 0.1    # 클수록 포인트 감소, 연산 속도 향상

# ── 3. Ground Removal ────────────────────────────────────────
GRID_CELL_SIZE  = 0.5   # 그리드 셀 크기 (m)
GROUND_DIFF     = 0.2   # 지면 판단 높이 차이 (m)

# ── 4. Euclidean Clustering ──────────────────────────────────
CLUSTER_RADIUS  = 0.3   # 클러스터 반경 (m)
MIN_CLUSTER_PTS = 3     # 클러스터 최소 포인트 수
MAX_CLUSTER_PTS = 2000  # 클러스터 최대 포인트 수

# ── 5. Bounding Box ──────────────────────────────────────────
BB_MIN_LWH = (0.5, 0.5, 0.5)   # 최소 크기 (m)
BB_MAX_LWH = (5.0, 5.0, 5.0)   # 최대 크기 (m)
```

> 파라미터 수정 후 `colcon build --packages-select lidar_detection` 재빌드 필요

---

## RViz2 시각화

```bash
rviz2
```

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/car1/scan/points` | PointCloud2 | 원본 LiDAR |
| `/preprocessed_points` | PointCloud2 | 전처리 완료 포인트클라우드 |
| `/bounding_boxes` | MarkerArray | 감지된 객체 바운딩 박스 |

Fixed Frame: `car1/lidar_link`

---
