import numpy as np
from typing import List


def get_fragment_pose(T_base_camera: np.ndarray,
                      T_camera_frag: np.ndarray) -> np.ndarray:
    """
    카메라가 검출한 조각 pose를 로봇 베이스 기준으로 변환.

    Args:
        T_base_camera : (4,4) Hand-Eye 캘리브레이션 결과 — 베이스 → 카메라
        T_camera_frag : (4,4) 카메라 기준 조각 pose — 카메라 → 조각 body

    Returns:
        T_base_frag   : (4,4) 베이스 기준 조각 pose
    """
    return T_base_camera @ T_camera_frag


def compute_nozzle_targets(waypoints: List[np.ndarray],
                           T_base_frag: np.ndarray,
                           T_EE_nozzle: np.ndarray) -> List[np.ndarray]:
    """
    조각 body frame 기준 waypoints를, 노즐 팁이 해당 위치에 오도록
    EE가 있어야 할 베이스 기준 pose 리스트로 변환.

    Args:
        waypoints    : Noether가 생성한 (4,4) pose 리스트 — 조각 body frame 기준
        T_base_frag  : (4,4) 베이스 기준 조각 pose (get_fragment_pose 출력)
        T_EE_nozzle  : (4,4) EE frame 기준 노즐 팁 pose — 노즐 캘리브레이션 결과

    Returns:
        targets : (4,4) pose 리스트 — 각 waypoint에서 EE가 있어야 할 베이스 기준 pose
    """
    T_nozzle_EE = np.linalg.inv(T_EE_nozzle)
    targets = []
    for wp in waypoints:
        T_base_EE = T_base_frag @ wp @ T_nozzle_EE
        targets.append(T_base_EE)
    return targets
