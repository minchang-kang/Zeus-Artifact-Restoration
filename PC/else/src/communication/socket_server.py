"""
Unix Domain Socket 서버 — Planning 프로세스(C++)로부터 Waypoints 수신.

프로토콜 (planning main.cpp과 동일):
  [uint32_t  N        ] — waypoint 총 수
  [double×16 × N bytes] — 각 Waypoint의 4×4 행렬 (row-major)
"""

import socket
import struct
import os
import numpy as np
from typing import List


class WaypointSocketServer:
    def __init__(self, socket_path: str = "/tmp/planning.sock"):
        self.socket_path = socket_path
        self._srv: socket.socket | None = None

    def start_listen(self) -> None:
        """소켓을 바인드하고 listen 상태로 만든다 (accept 전까지 블록 없음)."""
        if os.path.exists(self.socket_path):
            os.remove(self.socket_path)
        self._srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self._srv.bind(self.socket_path)
        self._srv.listen(1)
        print(f"[socket] listening on {self.socket_path}")

    def accept_and_receive(self) -> List[np.ndarray]:
        """planning 프로세스의 연결을 기다렸다가 waypoints를 수신하고 소켓을 닫는다."""
        if self._srv is None:
            raise RuntimeError("start_listen() 을 먼저 호출하세요")
        conn, _ = self._srv.accept()
        print("[socket] planning process connected")
        try:
            waypoints = self._recv_all(conn)
        finally:
            conn.close()
            self._srv.close()
            self._srv = None
            if os.path.exists(self.socket_path):
                os.remove(self.socket_path)
        print(f"[socket] received {len(waypoints)} waypoints")
        return waypoints

    def receive_once(self) -> List[np.ndarray]:
        """start_listen + accept_and_receive를 한번에 수행 (하위 호환)."""
        self.start_listen()
        return self.accept_and_receive()

    def _recv_all(self, conn: socket.socket) -> List[np.ndarray]:
        header = self._read_bytes(conn, 4)
        n = struct.unpack("<I", header)[0]   # uint32_t little-endian

        waypoints = []
        if n == 0:
            return waypoints

        data = self._read_bytes(conn, n * 16 * 8)  # N × 16 doubles
        raw = struct.unpack(f"<{n * 16}d", data)

        for i in range(n):
            mat = np.array(raw[i * 16:(i + 1) * 16], dtype=np.float64).reshape(4, 4)
            waypoints.append(mat)

        return waypoints

    @staticmethod
    def _read_bytes(conn: socket.socket, length: int) -> bytes:
        buf = b""
        while len(buf) < length:
            chunk = conn.recv(length - len(buf))
            if not chunk:
                raise ConnectionError("Connection closed before all data received")
            buf += chunk
        return buf
