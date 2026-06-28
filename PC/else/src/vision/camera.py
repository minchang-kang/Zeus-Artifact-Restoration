import time
import threading
from typing import Optional, Tuple, List, Dict, Any

import cv2
import numpy as np
import pyrealsense2 as rs

def simple_enhance_clahe(image: np.ndarray) -> np.ndarray:
    """Enhances image contrast using CLAHE on L channel.

    Applies Contrast Limited Adaptive Histogram Equalization to improve
    image quality for object detection.

    Args:
        image: Input image in BGR format (H, W, 3).

    Returns:
        Enhanced BGR image (H, W, 3).
    """
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=1, tileGridSize=(8, 8))
    l = clahe.apply(l)
    enhanced = cv2.merge([l, a, b])
    return cv2.cvtColor(enhanced, cv2.COLOR_LAB2BGR)


class RealSenseCapture:
    """RealSense D435i camera capture class.

    Handles RGB-D image capture with depth alignment and preprocessing.
    Camera position: wrist-mounted, 9cm from gripper center.
    """

    def __init__(
        self,
        serial_number: str = None,
        depth_w: int = 640,
        depth_h: int = 480,
        color_w: int = 640,
        color_h: int = 480,
        fps: int = 30
    ):
        """Initializes RealSense camera.

        Args:
            serial_number: Camera serial number (optional, for multi-camera).
            depth_w: Depth stream width.
            depth_h: Depth stream height.
            color_w: Color stream width.
            color_h: Color stream height.
            fps: Frame rate.

        Raises:
            RuntimeError: If camera stream configuration fails.
        """
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.align = None
        self.is_running = False
        self.depth_scale = None
        self.color_intrinsics = None
        self.depth_w, self.depth_h = depth_w, depth_h
        self.color_w, self.color_h = color_w, color_h
        self.fps = fps

        if serial_number:
            self.config.enable_device(serial_number)

        try:
            self.config.enable_stream(
                rs.stream.depth, self.depth_w, self.depth_h, rs.format.z16, self.fps
            )
            self.config.enable_stream(
                rs.stream.color, self.color_w, self.color_h, rs.format.bgr8, self.fps
            )
        except RuntimeError as e:
            print(f"Error enabling stream: {e}. Check if resolution/fps is supported.")
            self.pipeline = None
            raise

    def start(self) -> bool:
        """Starts the camera pipeline.

        Returns:
            True if started successfully, False otherwise.
        """
        if not self.pipeline:
            return False

        if not self.is_running:
            try:
                print("Starting RealSense pipeline...")
                profile = self.pipeline.start(self.config)
                self.is_running = True

                depth_sensor = profile.get_device().first_depth_sensor()
                self.depth_scale = depth_sensor.get_depth_scale()

                align_to = rs.stream.color
                self.align = rs.align(align_to)

                color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
                self.color_intrinsics = color_stream.get_intrinsics()
                print(f"RealSense started. Depth Scale: {self.depth_scale:.4f}")

                # Stabilization frames
                print("Waiting for frames to stabilize...")
                for _ in range(100):
                    self.pipeline.wait_for_frames()
                print("Stabilization complete.")
                return True

            except RuntimeError as e:
                print(f"Failed to start pipeline: {e}")
                self.is_running = False
                return False

        return True
    
    def capture_aligned_frames(self, timeout_ms: int = 5000):
        """Captures aligned RGB and depth frames.

        Args:
            timeout_ms: Frame wait timeout in milliseconds.

        Returns:
            Tuple of (color_image, depth_image) as numpy arrays.
            color_image: BGR format (H, W, 3), CLAHE enhanced.
            depth_image: uint16 raw depth values.
            Returns (None, None) on failure.
        """
        if not self.is_running:
            print("Error: Pipeline not running.")
            return None, None

        try:
            frames = self.pipeline.wait_for_frames(timeout_ms)
            if not frames:
                print("Error: Failed to receive frames (timeout).")
                return None, None

            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                print("Error: Failed to get valid aligned frames.")
                return None, None

            depth_image_raw = np.asanyarray(depth_frame.get_data())  # uint16
            color_image = np.asanyarray(color_frame.get_data())  # BGR

            # Apply CLAHE enhancement
            color_image = simple_enhance_clahe(color_image)

            # Filter out distant objects (>0.8m)
            threshold_m = 1.0
            mask = (depth_image_raw * self.depth_scale) >= threshold_m
            color_image[mask] = 0

            return color_image, depth_image_raw

        except Exception as e:
            print(f"Error during frame capture: {e}")
            return None, None

    def get_intrinsics(self):
        """Returns camera intrinsics."""
        return self.color_intrinsics

    def get_depth_scale(self) -> float:
        """Returns depth scale (meters per unit)."""
        return self.depth_scale

    def stop(self):
        """Stops the camera pipeline."""
        if self.is_running:
            print("Stopping RealSense pipeline...")
            self.pipeline.stop()
            self.is_running = False
            print("Pipeline stopped.")


class CameraManager:
    """Singleton manager for shared camera access.

    Provides a single camera instance shared across all modules,
    with frame caching support for consistent multi-tool operations.
    """

    _camera: Optional[RealSenseCapture] = None
    _cached_frame: Optional[Tuple[np.ndarray, np.ndarray, float]] = None
    _cached_mask: Optional[np.ndarray] = None

    @classmethod
    def get_camera(cls) -> RealSenseCapture:
        """Gets the shared camera instance, creating if needed.

        Returns:
            Shared RealSenseCapture instance.
        """
        if cls._camera is None:
            cls._camera = RealSenseCapture()
            cls._camera.start()
        return cls._camera

    @classmethod
    def capture_and_cache(cls) -> Tuple[np.ndarray, np.ndarray]:
        """Captures a frame and stores it in cache.

        Returns:
            Tuple of (color_image, depth_image).
        """
        camera = cls.get_camera()
        color, depth = camera.capture_aligned_frames()
        if color is not None and depth is not None:
            cls._cached_frame = (color, depth, time.time())
        return color, depth
    
    @classmethod
    def get_cached_frame(cls) -> Optional[Tuple[np.ndarray, np.ndarray, float]]:
        """Gets the cached frame from last capture.

        Returns:
            Tuple of (color, depth, timestamp) or None if no cache.
        """
        return cls._cached_frame
    
    @classmethod
    def cache_mask(cls, mask: np.ndarray):
        """Cache SAM mask for grasp detection."""
        cls._cached_mask = mask

    @classmethod
    def get_cached_mask(cls) -> Optional[np.ndarray]:
        """Get cached SAM mask."""
        return cls._cached_mask

    @classmethod
    def get_intrinsics(cls):
        """Gets camera intrinsics from shared instance."""
        return cls.get_camera().get_intrinsics()

    @classmethod
    def get_depth_scale(cls) -> float:
        """Gets depth scale from shared instance."""
        return cls.get_camera().get_depth_scale()

    @classmethod
    def release(cls):
        """Releases the shared camera instance."""
        if cls._camera is not None:
            cls._camera.stop()
            cls._camera = None
            cls._cached_frame = None
            cls._cached_mask = None

    @classmethod
    def start_preview(cls, window_name: str = "Camera Preview"):
        """Starts real-time camera preview window.

        Press 'q' to close the preview.
        Press 's' to save current frame.

        Args:
            window_name: Preview window title
        """
        camera = cls.get_camera()
        print(f"Starting camera preview. Press 'q' to close, 's' to save frame.")

        while True:
            color, depth = camera.capture_aligned_frames()
            if color is None:
                continue

            # Create depth colormap for visualization
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth, alpha=0.03),
                cv2.COLORMAP_JET
            )

            # Stack horizontally
            combined = np.hstack([color, depth_colormap])

            cv2.imshow(window_name, combined)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                timestamp = int(time.time())
                cv2.imwrite(f"/tmp/frame_{timestamp}_color.png", color)
                cv2.imwrite(f"/tmp/frame_{timestamp}_depth.png", depth)
                print(f"Saved frame to /tmp/frame_{timestamp}_*.png")

        cv2.destroyWindow(window_name)


class CameraPreviewThread:
    """Background thread for continuous camera preview with overlay support."""

    _instance: Optional["CameraPreviewThread"] = None
    _lock = threading.Lock()

    def __init__(self):
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._window_name = "Camera Preview"

        # Overlay data (thread-safe access via lock)
        self._overlay_lock = threading.Lock()
        self._detections: List[Dict[str, Any]] = []  # bbox, label, mask
        self._grasp_points: List[Dict[str, Any]] = []  # position, score

    @classmethod
    def get_instance(cls) -> "CameraPreviewThread":
        """Gets singleton instance."""
        with cls._lock:
            if cls._instance is None:
                cls._instance = CameraPreviewThread()
            return cls._instance

    def start(self):
        """Starts the preview thread."""
        if self._running:
            return

        self._running = True
        self._thread = threading.Thread(target=self._preview_loop, daemon=True)
        self._thread.start()
        print(f"[Preview] Started background camera preview")

    def stop(self):
        """Stops the preview thread."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
        cv2.destroyWindow(self._window_name)
        print("[Preview] Stopped")

    def is_running(self) -> bool:
        return self._running

    def update_detections(self, detections: List[Dict[str, Any]]):
        """Updates detection overlays (bboxes, labels, masks)."""
        with self._overlay_lock:
            self._detections = detections

    def update_grasp_points(self, grasps: List[Dict[str, Any]]):
        """Updates grasp point overlays."""
        with self._overlay_lock:
            self._grasp_points = grasps

    def clear_overlays(self):
        """Clears all overlays."""
        with self._overlay_lock:
            self._detections = []
            self._grasp_points = []

    def _preview_loop(self):
        """Main preview loop running in background thread."""
        camera = CameraManager.get_camera()

        while self._running:
            color, depth = camera.capture_aligned_frames()
            if color is None:
                time.sleep(0.01)
                continue

            # Make a copy to draw on
            display = color.copy()

            # Draw overlays
            with self._overlay_lock:
                self._draw_detections(display)
                self._draw_grasp_points(display)

            # Create depth colormap
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth, alpha=0.03),
                cv2.COLORMAP_JET
            )

            # Stack horizontally
            combined = np.hstack([display, depth_colormap])

            cv2.imshow(self._window_name, combined)

            # Non-blocking key check
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self._running = False
                break

        cv2.destroyWindow(self._window_name)

    def _draw_detections(self, image: np.ndarray):
        """Draws detection boxes and labels."""
        for det in self._detections:
            bbox = det.get("bbox")
            label = det.get("label", "")
            confidence = det.get("confidence", 0)
            mask = det.get("mask")

            # Draw mask overlay
            if mask is not None:
                overlay = image.copy()
                overlay[mask > 0] = [0, 255, 0]  # Green mask
                cv2.addWeighted(overlay, 0.3, image, 0.7, 0, image)

            # Draw bbox
            if bbox is not None:
                x1, y1, x2, y2 = [int(v) for v in bbox]
                cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                text = f"{label} ({confidence:.2f})"
                cv2.putText(image, text, (x1, y1 - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    def _draw_grasp_points(self, image: np.ndarray):
        """Draws grasp points on image."""
        intrinsics = CameraManager.get_intrinsics()
        if intrinsics is None:
            return

        for i, grasp in enumerate(self._grasp_points):
            pos = grasp.get("position")
            score = grasp.get("score", 0)

            if pos is None:
                continue

            # Project 3D point to 2D pixel
            x, y, z = pos
            if z <= 0:
                continue

            px = int(x * intrinsics.fx / z + intrinsics.ppx)
            py = int(y * intrinsics.fy / z + intrinsics.ppy)

            # Check bounds
            if 0 <= px < image.shape[1] and 0 <= py < image.shape[0]:
                # Best grasp = green, others = red gradient
                if i == 0:
                    color = (0, 255, 0)
                    radius = 12
                else:
                    color = (0, 0, 255)
                    radius = 8

                cv2.circle(image, (px, py), radius, color, -1)
                cv2.circle(image, (px, py), radius + 2, (255, 255, 255), 2)
                cv2.putText(image, f"G{i+1}:{score:.2f}", (px + 15, py),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)


if __name__ == "__main__":
    CameraManager.start_preview()
