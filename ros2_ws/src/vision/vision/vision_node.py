from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import time
from typing import Any

from ament_index_python.packages import get_package_share_directory
from bridge_interfaces.msg import VisionDetection, VisionDetectionArray
import cv2
import rclpy
from rclpy.node import Node

from vision.traffic_light import classify_traffic_light_color

try:
    from ultralytics import YOLO
except ImportError:  # pragma: no cover - handled at runtime with a clearer error
    YOLO = None


DEFAULT_MODEL_DIR = "yolo26n_ncnn_imgsz_640"


def _clamp_box(
    x: int,
    y: int,
    w: int,
    h: int,
    image_width: int,
    image_height: int,
) -> tuple[int, int, int, int] | None:
    x0 = max(0, x)
    y0 = max(0, y)
    x1 = min(image_width, x + w)
    y1 = min(image_height, y + h)
    width = x1 - x0
    height = y1 - y0
    if width <= 0 or height <= 0:
        return None
    return x0, y0, width, height


@dataclass
class DetectionAttribute:
    name: str
    value: str
    score: float


@dataclass
class DetectionRecord:
    class_name: str
    confidence: float
    x: int
    y: int
    width: int
    height: int
    attributes: list[DetectionAttribute] = field(default_factory=list)

    def add_attribute(self, name: str, value: str, score: float) -> None:
        self.attributes.append(DetectionAttribute(name=name, value=value, score=float(score)))


class VisionNode(Node):
    def __init__(self) -> None:
        super().__init__("vision_node")

        self._share_data_dir = Path(get_package_share_directory("vision")) / "data"
        self._source_data_dir = Path("/ros2_ws/src/vision/data")
        default_model_path = self._default_model_path()

        self.declare_parameter("camera_device", "/dev/video10")
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("camera_fps", 5.0)
        self.declare_parameter("process_rate_hz", 5.0)
        self.declare_parameter("model_path", str(default_model_path))
        self.declare_parameter("model_imgsz", 640)
        self.declare_parameter("confidence_threshold", 0.35)
        self.declare_parameter("iou_threshold", 0.7)
        self.declare_parameter("max_detections", 20)
        self.declare_parameter("class_filter", "traffic light,stop sign,person")
        self.declare_parameter("reconnect_delay_sec", 1.0)
        self.declare_parameter("log_interval_sec", 5.0)

        self._camera_device = str(self.get_parameter("camera_device").value)
        self._camera_width = int(self.get_parameter("camera_width").value)
        self._camera_height = int(self.get_parameter("camera_height").value)
        self._camera_fps = float(self.get_parameter("camera_fps").value)
        self._process_rate_hz = float(self.get_parameter("process_rate_hz").value)
        self._model_imgsz = int(self.get_parameter("model_imgsz").value)
        self._confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self._iou_threshold = float(self.get_parameter("iou_threshold").value)
        self._max_detections = int(self.get_parameter("max_detections").value)
        self._class_filter = str(self.get_parameter("class_filter").value)
        self._reconnect_delay_sec = max(0.1, float(self.get_parameter("reconnect_delay_sec").value))
        self._log_interval_sec = max(1.0, float(self.get_parameter("log_interval_sec").value))

        self._publisher = self.create_publisher(VisionDetectionArray, "/vision/detections", 10)
        self._capture: cv2.VideoCapture | None = None
        self._camera_connected = False
        self._last_loop_summary = 0.0

        model_path = self._resolve_model_path(str(self.get_parameter("model_path").value))
        self._model = self._load_model(model_path)
        self._model_names = self._load_model_names(self._model)
        self._class_filter_ids = self._resolve_class_filter(self._class_filter)

        self.get_logger().info(
            "Loaded Ultralytics model path=%s imgsz=%d classes=%d filter=%s"
            % (
                model_path,
                self._model_imgsz,
                len(self._model_names),
                self._class_filter or "all",
            )
        )

    def _default_model_path(self) -> Path:
        source_model_path = self._source_data_dir / DEFAULT_MODEL_DIR
        if source_model_path.is_dir():
            return source_model_path
        return self._share_data_dir / DEFAULT_MODEL_DIR

    def _resolve_model_path(self, raw_path: str) -> str:
        raw_path = raw_path.strip()
        if not raw_path:
            raise ValueError("model_path cannot be empty")

        path = Path(raw_path).expanduser()
        if path.is_absolute():
            if not path.exists():
                raise FileNotFoundError(f"Ultralytics model path not found: {path}")
            return str(path)

        candidates = [
            Path.cwd() / path,
            self._source_data_dir / path,
            self._share_data_dir / path,
        ]
        for candidate in candidates:
            if candidate.exists():
                return str(candidate)

        # Allow explicit Ultralytics model names such as yolo26n.pt for quick
        # experiments. Classroom deployment should use a checked-in/exported
        # model directory so startup never depends on network access.
        if "/" not in raw_path and "\\" not in raw_path:
            return raw_path

        raise FileNotFoundError(
            "Ultralytics model path not found: %s. Put exported models under "
            "ros2_ws/src/vision/data or pass model_path:=/absolute/path." % raw_path
        )

    def _load_model(self, model_path: str) -> Any:
        if YOLO is None:
            raise RuntimeError(
                "The vision node requires the 'ultralytics' Python package. "
                "Install the project Docker image or run: pip install ultralytics==8.4.41 ncnn"
            )
        return YOLO(model_path)

    def _load_model_names(self, model: Any) -> dict[int, str]:
        names = model.names
        if isinstance(names, dict):
            return {int(index): str(name) for index, name in names.items()}
        return {int(index): str(name) for index, name in enumerate(names)}

    def _resolve_class_filter(self, raw_filter: str) -> list[int] | None:
        requested = {name.strip().lower() for name in raw_filter.split(",") if name.strip()}
        if not requested:
            return None

        indexes = [
            index
            for index, name in self._model_names.items()
            if name.strip().lower() in requested
        ]
        available = {self._model_names[index].strip().lower() for index in indexes}
        missing = sorted(requested - available)
        if missing:
            self.get_logger().warn(
                "Vision class_filter entries not found in model: %s"
                % ", ".join(missing)
            )
        return indexes

    def _release_camera(self) -> None:
        if self._capture is not None:
            self._capture.release()
            self._capture = None
        if self._camera_connected:
            self._camera_connected = False
            self.get_logger().warn("Vision camera disconnected; waiting for %s" % self._camera_device)

    def _ensure_camera(self) -> bool:
        if self._capture is not None and self._capture.isOpened():
            return True

        self._release_camera()
        capture = cv2.VideoCapture(self._camera_device, cv2.CAP_V4L2)
        if not capture.isOpened():
            self.get_logger().warn(
                "Waiting for camera device %s"
                % self._camera_device,
                throttle_duration_sec=self._log_interval_sec,
            )
            capture.release()
            time.sleep(self._reconnect_delay_sec)
            return False

        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if self._camera_width > 0:
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, float(self._camera_width))
        if self._camera_height > 0:
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self._camera_height))
        if self._camera_fps > 0.0:
            capture.set(cv2.CAP_PROP_FPS, self._camera_fps)

        self._capture = capture
        self._camera_connected = True
        self.get_logger().info(
            "Connected vision camera %s at requested %dx%d @ %.1f fps"
            % (self._camera_device, self._camera_width, self._camera_height, self._camera_fps)
        )
        return True

    def _infer(self, frame) -> list[DetectionRecord]:
        result = self._model.predict(
            frame,
            imgsz=self._model_imgsz,
            conf=self._confidence_threshold,
            iou=self._iou_threshold,
            classes=self._class_filter_ids,
            max_det=self._max_detections,
            verbose=False,
        )[0]
        return self._decode_detections(result, frame.shape[1], frame.shape[0])

    def _decode_detections(
        self,
        result: Any,
        image_width: int,
        image_height: int,
    ) -> list[DetectionRecord]:
        records: list[DetectionRecord] = []
        boxes = result.boxes
        if boxes is None:
            return records

        for box in boxes:
            class_id = int(box.cls.item())
            confidence = float(box.conf.item())
            x1, y1, x2, y2 = [int(round(value)) for value in box.xyxy[0].tolist()]
            clamped = _clamp_box(
                x=x1,
                y=y1,
                w=x2 - x1,
                h=y2 - y1,
                image_width=image_width,
                image_height=image_height,
            )
            if clamped is None:
                continue

            x, y, width, height = clamped
            records.append(
                DetectionRecord(
                    class_name=self._model_names.get(class_id, f"class_{class_id}"),
                    confidence=confidence,
                    x=x,
                    y=y,
                    width=width,
                    height=height,
                )
            )
        return records

    def _build_detection_msg(self, record: DetectionRecord) -> VisionDetection:
        detection = VisionDetection()
        detection.class_name = record.class_name
        detection.confidence = float(record.confidence)
        detection.x = int(record.x)
        detection.y = int(record.y)
        detection.width = int(record.width)
        detection.height = int(record.height)
        for attribute in record.attributes:
            detection.attribute_names.append(attribute.name)
            detection.attribute_values.append(attribute.value)
            detection.attribute_scores.append(float(attribute.score))
        return detection

    def _build_detection_array_msg(
        self,
        capture_stamp,
        image_width: int,
        image_height: int,
        records: list[DetectionRecord],
    ) -> VisionDetectionArray:
        message = VisionDetectionArray()
        message.header.stamp = capture_stamp
        message.header.frame_id = "vision_camera"
        message.image_width = int(image_width)
        message.image_height = int(image_height)
        for record in records:
            message.detections.append(self._build_detection_msg(record))
        return message

    def run(self) -> None:
        period = 1.0 / self._process_rate_hz if self._process_rate_hz > 0.0 else 0.0
        next_cycle = time.monotonic()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            if not self._ensure_camera():
                next_cycle = time.monotonic()
                continue

            if period > 0.0:
                sleep_time = next_cycle - time.monotonic()
                if sleep_time > 0.0:
                    time.sleep(sleep_time)

            ok, frame = self._capture.read()
            if not ok or frame is None:
                self.get_logger().warn(
                    "Failed to read a frame from %s; reconnecting"
                    % self._camera_device,
                    throttle_duration_sec=self._log_interval_sec,
                )
                self._release_camera()
                time.sleep(self._reconnect_delay_sec)
                next_cycle = time.monotonic()
                continue

            capture_stamp = self.get_clock().now().to_msg()
            inference_start = time.monotonic()
            try:
                detections = self._infer(frame)

                for detection in detections:
                    object_crop = frame[
                        detection.y : detection.y + detection.height,
                        detection.x : detection.x + detection.width,
                    ]

                    if detection.class_name == "traffic light":
                        traffic_light_crop = object_crop
                        color_label, color_score = classify_traffic_light_color(traffic_light_crop)
                        detection.add_attribute("color", color_label, color_score)
                    elif detection.class_name == "face":
                        face_crop = object_crop
                        # TODO(student): analyze the face crop and attach your own
                        # attributes here, for example gender or customer type.
                        _ = face_crop
                        pass
                    elif detection.class_name == "my_object":
                        custom_object_crop = object_crop
                        # TODO(student): add custom object-specific checks here.
                        _ = custom_object_crop
                        pass

                message = self._build_detection_array_msg(
                    capture_stamp=capture_stamp,
                    image_width=frame.shape[1],
                    image_height=frame.shape[0],
                    records=detections,
                )
                self._publisher.publish(message)
                detection_count = len(message.detections)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"Vision inference failed for one frame: {exc}")
                detection_count = 0
            inference_ms = (time.monotonic() - inference_start) * 1000.0

            now = time.monotonic()
            if now - self._last_loop_summary >= self._log_interval_sec:
                self._last_loop_summary = now
                self.get_logger().info(
                    "Vision frame %dx%d inference=%.1fms detections=%d target_rate=%.1fHz"
                    % (
                        frame.shape[1],
                        frame.shape[0],
                        inference_ms,
                        detection_count,
                        self._process_rate_hz,
                    )
                )

            if period > 0.0:
                next_cycle += period
                if next_cycle < time.monotonic():
                    next_cycle = time.monotonic()

        self._release_camera()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
