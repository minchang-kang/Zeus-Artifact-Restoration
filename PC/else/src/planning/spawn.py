import subprocess


def spawn_planning(executable: str, pcd: str, socket: str,
                   radius: float = 0.01, spacing: float = 0.005,
                   segment: str = "none") -> subprocess.Popen:
    """
    C++ planning 프로세스를 백그라운드에서 실행하고 Popen 객체를 반환.

    호출자가 반환된 Popen으로 종료 대기(proc.wait()) 또는
    종료 코드 확인(proc.returncode) 등을 직접 처리한다.
    """
    cmd = [
        executable,
        "--input",   pcd,
        "--socket",  socket,
        "--radius",  str(radius),
        "--spacing", str(spacing),
        "--segment", segment,
    ]
    print(f"[planning] spawn: {' '.join(cmd)}")
    return subprocess.Popen(cmd)
