#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LiDAR 처리 실습 파이프라인
==========================
처리 순서:
  1. ROI 필터링     - 관심 영역(직육면체)만 남기기
  2. 다운샘플링     - 복셀 그리드로 포인트 수 줄이기
  3. 지면 제거      - 그리드 셀 기반으로 바닥 포인트 제거
  4. 클러스터링     - 유클리드 거리 기반으로 객체 분리
  5. 바운딩 박스    - 각 클러스터에 3D 박스 씌우기

구독 토픽 : /<car_name>/scan/points  (sensor_msgs/PointCloud2)
발행 토픽 : /preprocessed_points     (sensor_msgs/PointCloud2)   ← RViz 시각화용
            /bounding_boxes           (visualization_msgs/MarkerArray)
"""

import rclpy
from rclpy.node import Node

import numpy as np
from scipy.spatial import cKDTree

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header


# ============================================================
#  파라미터 설정  ←  여기 값을 바꿔가며 실습하세요!
# ============================================================

# ── 1. ROI (Region of Interest) ─────────────────────────────
# 단위: meter  /  차량 기준 좌표계 (X=전방, Y=좌측, Z=위)
ROI_X_MIN, ROI_X_MAX =  -5.0, 20.0   # 전방 거리
ROI_Y_MIN, ROI_Y_MAX = -5.0,  5.0   # 좌우 너비
ROI_Z_MIN, ROI_Z_MAX = -2.0,  1.5   # 높이 범위

# ── 2. Voxel Downsampling ────────────────────────────────────
VOXEL_SIZE = 0.1    # 복셀 한 변의 길이 (m). 클수록 포인트가 줄어듦

# ── 3. Ground Removal ────────────────────────────────────────
GRID_CELL_SIZE  = 0.5   # 그리드 셀 한 변 크기 (m)
GROUND_DIFF     = 0.2   # 셀 최저점 + 이 값 이하이면 지면으로 판단 (m)
GROUND_MIN_PTS  = 3     # 셀에 포인트가 이 수 미만이면 지면 판단 보류

# ── 4. Euclidean Clustering ──────────────────────────────────
CLUSTER_RADIUS  = 0.3   # 같은 클러스터로 묶는 최대 거리 (m)
MIN_CLUSTER_PTS = 3     # 클러스터 최소 포인트 수 (이하면 노이즈로 버림)
MAX_CLUSTER_PTS = 2000  # 클러스터 최대 포인트 수 (이상이면 너무 큰 덩어리)

# ── 5. Bounding Box 크기 필터 ────────────────────────────────
BB_MIN_LWH = (0.5, 0.5, 0.5)   # (길이, 너비, 높이) 최솟값 (m)
BB_MAX_LWH = (5.0, 5.0, 5.0)   # (길이, 너비, 높이) 최댓값 (m)


# ============================================================
#  처리 함수
# ============================================================

def roi_filter(pts: np.ndarray) -> np.ndarray:
    """
    1단계: ROI 필터링
    -----------------
    지정한 직육면체 범위(ROI) 밖의 포인트를 모두 제거합니다.
    범위를 좁힐수록 연산이 빨라지지만, 멀리 있는 객체를 놓칩니다.
    """
    mask = (
        (ROI_X_MIN <= pts[:, 0]) & (pts[:, 0] <= ROI_X_MAX) &
        (ROI_Y_MIN <= pts[:, 1]) & (pts[:, 1] <= ROI_Y_MAX) &
        (ROI_Z_MIN <= pts[:, 2]) & (pts[:, 2] <= ROI_Z_MAX)
    )
    return pts[mask]


def voxel_downsample(pts: np.ndarray) -> np.ndarray:
    """
    2단계: Voxel Downsampling
    -------------------------
    공간을 VOXEL_SIZE 크기의 격자(복셀)로 나누고,
    각 격자 안에 있는 포인트 중 하나만 대표로 남깁니다.
    포인트 수를 줄여 이후 처리 속도를 높입니다.
    """
    if len(pts) == 0:
        return pts
    # 각 포인트가 속하는 복셀 인덱스 계산
    voxel_idx = np.floor(pts / VOXEL_SIZE).astype(np.int32)
    # 중복 복셀 제거 → 첫 번째 포인트만 유지
    _, unique_idx = np.unique(voxel_idx, axis=0, return_index=True)
    return pts[unique_idx]


def remove_ground(pts: np.ndarray) -> np.ndarray:
    """
    3단계: Ground Removal (그리드 기반)
    ------------------------------------
    XY 평면을 GRID_CELL_SIZE 크기의 격자로 나눕니다.
    각 격자 셀에서 가장 낮은 Z값을 '지면 높이'로 봅니다.
    포인트의 Z값이 (지면 높이 + GROUND_DIFF) 이하이면 지면으로 제거합니다.
    """
    if len(pts) == 0:
        return pts

    # 각 포인트의 그리드 셀 번호 계산
    cx = np.floor((pts[:, 0] - ROI_X_MIN) / GRID_CELL_SIZE).astype(np.int32)
    cy = np.floor((pts[:, 1] - ROI_Y_MIN) / GRID_CELL_SIZE).astype(np.int32)
    cell_key = cx * 100_000 + cy   # (cx, cy) 쌍을 하나의 정수 키로 인코딩

    # 셀별 최저 Z와 포인트 수 계산
    min_z  = {}
    counts = {}
    for key, z in zip(cell_key, pts[:, 2]):
        if key not in min_z or z < min_z[key]:
            min_z[key] = z
        counts[key] = counts.get(key, 0) + 1

    # 지면 판단: 충분한 포인트가 있고, 최저점 근처이면 지면
    is_ground = np.array([
        counts.get(k, 0) >= GROUND_MIN_PTS and
        (pts[i, 2] - min_z.get(k, -9999.0)) < GROUND_DIFF
        for i, k in enumerate(cell_key)
    ])
    return pts[~is_ground]


def euclidean_clustering(pts: np.ndarray) -> list:
    """
    4단계: Euclidean Clustering
    ---------------------------
    KD-Tree로 각 포인트 주변 CLUSTER_RADIUS 안의 이웃을 찾고,
    BFS(너비 우선 탐색)로 연결된 포인트들을 같은 클러스터로 묶습니다.
    MIN/MAX_CLUSTER_PTS 범위를 벗어나는 클러스터는 버립니다.
    """
    if len(pts) == 0:
        return []

    tree    = cKDTree(pts[:, :2])   # XY 평면 기준 KD-Tree
    visited = np.zeros(len(pts), dtype=bool)
    clusters = []

    for i in range(len(pts)):
        if visited[i]:
            continue

        # 이웃 탐색
        neighbors = tree.query_ball_point(pts[i, :2], CLUSTER_RADIUS)
        if len(neighbors) < MIN_CLUSTER_PTS:
            visited[i] = True
            continue

        # BFS로 클러스터 확장
        cluster_idx = set()
        queue = list(neighbors)
        while queue:
            cur = queue.pop()
            if visited[cur]:
                continue
            visited[cur] = True
            cluster_idx.add(cur)
            # 현재 포인트 주변도 재탐색
            new_neighbors = tree.query_ball_point(pts[cur, :2], CLUSTER_RADIUS)
            if len(new_neighbors) >= MIN_CLUSTER_PTS:
                queue.extend(new_neighbors)

        # 크기 필터 적용
        if MIN_CLUSTER_PTS <= len(cluster_idx) <= MAX_CLUSTER_PTS:
            clusters.append(pts[list(cluster_idx)])

    return clusters


def compute_bounding_box(cluster: np.ndarray):
    """
    5단계: 바운딩 박스 계산
    -----------------------
    클러스터 포인트의 min/max로 축-정렬 바운딩 박스(AABB)를 계산합니다.
    반환값: (center, size) - 모두 (x, y, z) numpy 배열
    """
    min_pt = cluster.min(axis=0)
    max_pt = cluster.max(axis=0)
    center = (min_pt + max_pt) / 2.0
    size   = max_pt - min_pt
    return center, size


def is_valid_bbox(size: np.ndarray) -> bool:
    """바운딩 박스 크기가 유효한지 확인 (너무 작거나 큰 객체 제거)"""
    l, w, h = size[0], size[1], size[2]
    return (
        BB_MIN_LWH[0] < l < BB_MAX_LWH[0] and
        BB_MIN_LWH[1] < w < BB_MAX_LWH[1] and
        BB_MIN_LWH[2] < h < BB_MAX_LWH[2]
    )


# ============================================================
#  ROS 2 노드
# ============================================================

class LidarPipelineNode(Node):

    def __init__(self):
        super().__init__('lidar_pipeline')

        # 차량 이름 파라미터 (기본값: car1)
        self.declare_parameter('car_name', 'car1')
        car_name = self.get_parameter('car_name').get_parameter_value().string_value

        lidar_topic   = f'/{car_name}/scan/points'
        self.frame_id = f'{car_name}/lidar_link'

        # 구독자 / 발행자
        self.sub = self.create_subscription(
            PointCloud2, lidar_topic, self.callback, 10)
        self.pub_cloud = self.create_publisher(
            PointCloud2, '/preprocessed_points', 10)
        self.pub_boxes = self.create_publisher(
            MarkerArray, '/bounding_boxes', 10)

        self.get_logger().info(f'구독 토픽: {lidar_topic}')
        self.get_logger().info('LiDAR Pipeline 노드 시작')

    def callback(self, msg: PointCloud2):
        # PointCloud2 → numpy (N×3)
        # read_points()는 구조체 배열을 반환하므로 필드를 따로 추출
        cloud = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        pts = np.column_stack([cloud['x'], cloud['y'], cloud['z']]).astype(np.float32)
        if pts.size == 0:
            return

        # ── 파이프라인 ──────────────────────────────────────
        pts      = roi_filter(pts)              # 1. ROI
        pts      = voxel_downsample(pts)        # 2. 다운샘플링
        pts      = remove_ground(pts)           # 3. 지면 제거
        clusters = euclidean_clustering(pts)    # 4. 클러스터링
        # ────────────────────────────────────────────────────

        self._publish_cloud(pts, msg.header.stamp)
        self._publish_boxes(clusters, msg.header.stamp)

    def _publish_cloud(self, pts: np.ndarray, stamp):
        """전처리 완료된 PointCloud2 발행 (RViz 확인용)"""
        header = Header()
        header.stamp    = stamp
        header.frame_id = self.frame_id
        self.pub_cloud.publish(pc2.create_cloud_xyz32(header, pts.tolist()))

    def _publish_boxes(self, clusters: list, stamp):
        """바운딩 박스 MarkerArray 발행"""
        markers = MarkerArray()

        # 이전 프레임 마커 전체 삭제
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        valid_id = 0
        for cluster in clusters:
            center, size = compute_bounding_box(cluster)
            if not is_valid_bbox(size):
                continue
            markers.markers.append(
                self._make_box_marker(valid_id, center, size, stamp))
            valid_id += 1

        self.pub_boxes.publish(markers)

    def _make_box_marker(self, marker_id, center, size, stamp) -> Marker:
        """CUBE 타입 마커 생성"""
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp    = stamp
        m.ns     = 'bounding_boxes'
        m.id     = marker_id
        m.type   = Marker.CUBE
        m.action = Marker.ADD

        m.pose.position.x = float(center[0])
        m.pose.position.y = float(center[1])
        m.pose.position.z = float(center[2])
        m.pose.orientation.w = 1.0

        m.scale.x = float(size[0])
        m.scale.y = float(size[1])
        m.scale.z = float(size[2])

        # 반투명 초록색
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.3
        m.color.a = 0.4

        return m


# ============================================================
#  진입점
# ============================================================

def main(args=None):
    rclpy.init(args=args)
    node = LidarPipelineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
