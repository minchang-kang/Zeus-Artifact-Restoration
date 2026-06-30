"""
Zeus Artifact Restoration — else 프로세스 진입점.

실행 흐름:
  1. Unix Socket으로 Planning 프로세스(C++)로부터 Waypoints 수신
  2. 각 Waypoint에 대해 Tesseract IK 풀기
  3. (추후) Joint Angle 배열 → PLC TCP/IP 전송
"""

import sys
import os
import yaml
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))

from communication.socket_server import WaypointSocketServer
from motion.ik_solver import IKSolver

CONFIG_PATH = os.path.join(os.path.dirname(__file__), "config", "config.yaml")


def load_config(path: str = CONFIG_PATH) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def main():
    cfg = load_config()
    k = cfg["kinematics"]
    p = cfg["planning"]

    solver = IKSolver(
        urdf_path=k["urdf"],
        srdf_path=k["srdf"],
        group_name=k["group"],
        tip_link=k["tip_link"],
        working_frame=k["base_link"],
    )

    server = WaypointSocketServer(p["socket"])
    waypoints = server.receive_once()

    if not waypoints:
        print("[main] no waypoints received")
        return

    joint_solutions = []
    failed = 0
    for i, wp in enumerate(waypoints):
        q = solver.solve(wp)
        if q is None:
            print(f"[ik] waypoint {i}: no solution")
            failed += 1
        else:
            joint_solutions.append(q)
            print(f"[ik] waypoint {i}: {np.degrees(q).round(2)} deg")

    print(f"\n[done] solved {len(joint_solutions)}/{len(waypoints)} ({failed} failed)")

    # TODO: joint_solutions → PLC TCP/IP 전송


if __name__ == "__main__":
    main()
