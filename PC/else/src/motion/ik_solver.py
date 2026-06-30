"""
Tesseract Python 기반 IK 솔버.

사용 순서:
  1. IKSolver(urdf_path, srdf_path) 생성
  2. solve(target_pose_4x4) 호출 → joint angles (6,) ndarray 반환
  3. verify_fk(joint_angles) 로 역검증 (선택)

URDF/SRDF 준비 전에는 IKSolver 생성 시 FileNotFoundError가 발생한다.
"""

import os
import numpy as np
from typing import Optional

from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_common import GeneralResourceLocator, FilesystemPath
from tesseract_robotics.tesseract_urdf import parseURDFFile
from tesseract_robotics.tesseract_srdf import SRDFModel
from tesseract_robotics.tesseract_kinematics import KinGroupIKInput


class IKSolver:
    def __init__(
        self,
        urdf_path: str,
        srdf_path: str,
        group_name: str = "manipulator",
        tip_link: str = "tool0",
        working_frame: str = "base_link",
    ):
        if not os.path.isfile(urdf_path):
            raise FileNotFoundError(f"URDF not found: {urdf_path}")
        if not os.path.isfile(srdf_path):
            raise FileNotFoundError(f"SRDF not found: {srdf_path}")

        self.group_name = group_name
        self.tip_link = tip_link
        self.working_frame = working_frame

        locator = GeneralResourceLocator()
        # URDF 디렉토리를 리소스 검색 경로로 등록 (메시 파일 등 상대 경로 해결용)
        locator.addPath(FilesystemPath(os.path.dirname(os.path.abspath(urdf_path))))

        scene_graph = parseURDFFile(urdf_path, locator)
        if scene_graph is None:
            raise RuntimeError(f"Failed to parse URDF: {urdf_path}")

        srdf_model = SRDFModel()
        srdf_model.initFile(scene_graph, srdf_path, locator)

        self._env = Environment()
        if not self._env.init(scene_graph, srdf_model.kinematics_information):
            raise RuntimeError("Tesseract Environment initialization failed")

        print(f"[ik] environment initialized — group: {group_name}")

    def solve(
        self, target_pose: np.ndarray, seed: Optional[np.ndarray] = None
    ) -> Optional[np.ndarray]:
        """
        target_pose: (4,4) ndarray — Isometry3d (절단면 Waypoint, base_link 기준)
        seed:        (N,)  ndarray — 초기 관절각 추정값 (없으면 영벡터)
        반환:        (N,)  ndarray — 첫 번째 IK 해, 실패 시 None
        """
        kin_group = self._env.getKinematicGroup(self.group_name)

        n_joints = len(kin_group.getJointNames())
        if seed is None:
            seed = np.zeros(n_joints)

        ik_input = KinGroupIKInput(target_pose, self.working_frame, self.tip_link)
        solutions = kin_group.calcInvKin(ik_input, seed)

        if not solutions:
            return None

        return np.array(solutions[0])

    def verify_fk(self, joint_angles: np.ndarray) -> np.ndarray:
        """
        Forward Kinematics 역검증.
        반환: tip_link의 4×4 pose (base_link 기준)
        """
        kin_group = self._env.getKinematicGroup(self.group_name)
        link_transforms = kin_group.calcFwdKin(joint_angles)
        return np.array(link_transforms[self.tip_link])
