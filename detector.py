"""
detector.py
-----------
Real-time fire / open-flame detection module using Ultralytics YOLO.
Designed for the fire-fighting robotic car and student understanding.

Pipeline:
    OpenCV Frame -> YOLO Inference -> Target Class Filtering (Fire / Flame / Smoke)
    -> Temporal Confirmation -> Direction & Geometry Calculation (cx, cy, area, zone)
"""

import os
import urllib.request
import torch
from ultralytics import YOLO


class FireDetector:
    """
    FireDetector for real-time fire and open-flame detection.

    Supports:
        - GPU acceleration (CUDA device 0 on RTX 3050) or CPU fallback
        - Models with class names 'fire', 'open_flame', 'flame', and 'smoke'
        - Temporal multi-frame filtering to reject transient false alarms
        - Spatial geometry calculation (center coordinates, apparent area, LEFT/CENTER/RIGHT direction)
    """

    FALLBACK_MODEL_URL = (
        "https://huggingface.co/rabahdev/fire-smoke-yolov8n/resolve/main/best.pt"
    )

    def __init__(
        self,
        model_path="models/fire_weights.pt",
        conf_threshold=0.25,
        consecutive_frames_required=4,
        detect_smoke=False,
        device="auto",
    ):
        """
        :param model_path: Path to the YOLO model weights (.pt).
        :param conf_threshold: Minimum confidence score (0.0 to 1.0) to accept a detection.
        :param consecutive_frames_required: Number of consecutive positive frames to confirm flame.
        :param detect_smoke: Whether to also detect smoke alongside fire (default: False).
        :param device: 'auto' (selects GPU if available, else CPU), 0 (GPU), or 'cpu'.
        """
        self.model_path = model_path
        self.conf_threshold = conf_threshold
        self.consecutive_frames_required = consecutive_frames_required
        self.detect_smoke = detect_smoke

        # --------------------------------------------------
        # 1. Device Selection (GPU / CPU)
        # --------------------------------------------------
        if device == "auto":
            if torch.cuda.is_available():
                self.device = 0
                gpu_name = torch.cuda.get_device_name(0)
                print(f"[INFO] CUDA available. Using GPU (device=0): {gpu_name}")
            else:
                self.device = "cpu"
                print("[WARNING] CUDA not available. Using CPU (device='cpu').")
        else:
            self.device = device
            print(f"[INFO] Using explicitly specified device: {self.device}")

        # --------------------------------------------------
        # 2. Ensure model weights exist locally
        # --------------------------------------------------
        self._ensure_model_exists()

        # --------------------------------------------------
        # 3. Load YOLO model
        # --------------------------------------------------
        print(f"[INFO] Loading YOLO fire detection model from: {self.model_path}")
        self.model = YOLO(self.model_path)

        # Print model classes
        if isinstance(self.model.names, dict):
            print(f"[INFO] Model classes: {self.model.names}")
        else:
            print(f"[INFO] Model classes: {dict(enumerate(self.model.names))}")

        # --------------------------------------------------
        # 4. Target Classes Configuration
        # --------------------------------------------------
        # Models may name flame classes as 'fire', 'open_flame', or 'flame'.
        # We accept any of these aliases to ensure compatibility with both
        # fire_weights.pt ({0: 'fire', 1: 'smoke'}) and Thalos/D-Fire models.
        self.fire_aliases = {"fire", "open_flame", "flame", "fire_risk"}
        self.smoke_aliases = {"smoke"}

        self.target_classes = set(self.fire_aliases)
        if self.detect_smoke:
            self.target_classes.update(self.smoke_aliases)

        print(f"[INFO] Active target class filters: {list(self.target_classes)}")

        # --------------------------------------------------
        # 5. Temporal Filtering State
        # --------------------------------------------------
        self.consecutive_fire_frames = 0
        self.fire_confirmed = False

    def _ensure_model_exists(self):
        """Checks if model weights file exists; downloads fallback if missing."""
        if not os.path.exists(self.model_path):
            parent_dir = os.path.dirname(self.model_path)
            if parent_dir:
                os.makedirs(parent_dir, exist_ok=True)

            # If fire_weights.pt is missing but best.pt exists, use best.pt
            if self.model_path.endswith("fire_weights.pt") and os.path.exists("models/best.pt"):
                print(f"[WARNING] '{self.model_path}' not found, falling back to 'models/best.pt'.")
                self.model_path = "models/best.pt"
                return

            print(f"[INFO] Model weights not found at '{self.model_path}'.")
            print("[INFO] Downloading pre-trained YOLO fire model from Hugging Face...")
            urllib.request.urlretrieve(self.FALLBACK_MODEL_URL, self.model_path)
            print(f"[INFO] Model downloaded successfully to '{self.model_path}'.")

    def reset(self):
        """Resets the temporal confirmation filter."""
        self.consecutive_fire_frames = 0
        self.fire_confirmed = False

    def _get_class_name(self, class_id):
        """Safely retrieves the lowercase class name from the model."""
        if isinstance(self.model.names, dict):
            return str(self.model.names.get(class_id, class_id)).lower()
        if 0 <= class_id < len(self.model.names):
            return str(self.model.names[class_id]).lower()
        return str(class_id).lower()

    def detect(self, frame):
        """
        Run inference on an OpenCV BGR frame.

        :param frame: numpy.ndarray, input image frame from webcam
        :return: dict with keys:
                 - 'detections': list of dicts for each detected flame/fire region
                 - 'strongest': dict of the fire with highest confidence, or None
                 - 'confirmed': bool (True if fire persisted for N consecutive frames)
                 - 'consecutive_count': int (current consecutive frame streak)
                 - 'fire_direction': 'LEFT', 'CENTER', 'RIGHT', or 'NONE'
        """
        height, width = frame.shape[:2]

        # Run YOLO inference on selected device (device=0 for GPU, 'cpu' for CPU)
        results = self.model.predict(
            source=frame,
            conf=self.conf_threshold,
            imgsz=640,
            device=self.device,
            verbose=False,
        )

        detections = []

        if len(results) > 0 and results[0].boxes is not None:
            boxes = results[0].boxes
            for box in boxes:
                class_id = int(box.cls[0].item())
                raw_class_name = self._get_class_name(class_id)

                # Check if this class matches our target classes (fire/flame or smoke)
                if raw_class_name in self.target_classes:
                    conf = float(box.conf[0].item())

                    # Bounding box coordinates [x1, y1, x2, y2]
                    xyxy = box.xyxy[0].cpu().numpy()
                    x1, y1, x2, y2 = map(int, xyxy)

                    # Clamp coordinates within frame boundaries
                    x1 = max(0, min(x1, width - 1))
                    y1 = max(0, min(y1, height - 1))
                    x2 = max(0, min(x2, width - 1))
                    y2 = max(0, min(y2, height - 1))

                    # Bounding box dimensions & area
                    box_w = max(0, x2 - x1)
                    box_h = max(0, y2 - y1)
                    area = box_w * box_h

                    # Center coordinates
                    # cx = (x1 + x2) / 2
                    # cy = (y1 + y2) / 2
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)

                    # Direction calculation:
                    # LEFT:   0 to (width / 3)
                    # CENTER: (width / 3) to (2 * width / 3)
                    # RIGHT:  (2 * width / 3) to width
                    left_boundary = width / 3.0
                    right_boundary = 2.0 * width / 3.0

                    if cx < left_boundary:
                        direction = "LEFT"
                    elif cx < right_boundary:
                        direction = "CENTER"
                    else:
                        direction = "RIGHT"

                    # Normalize display label (FIRE or SMOKE)
                    display_label = "SMOKE" if raw_class_name in self.smoke_aliases else "FIRE"

                    detections.append({
                        "bbox": (x1, y1, x2, y2),
                        "confidence": conf,
                        "class_name": display_label,
                        "raw_class": raw_class_name,
                        "center": (cx, cy),
                        "width": box_w,
                        "height": box_h,
                        "area": area,
                        "direction": direction,
                    })

        # --- Temporal Multi-Frame Filtering ---
        # Require positive fire detection across several consecutive frames to confirm
        # Only count detections that are actual fire (ignore standalone smoke for fire confirmation)
        has_fire = any(d["class_name"] == "FIRE" for d in detections)

        if has_fire:
            self.consecutive_fire_frames += 1
        else:
            self.consecutive_fire_frames = 0

        self.fire_confirmed = (
            self.consecutive_fire_frames >= self.consecutive_frames_required
        )

        # Identify strongest detection (highest confidence)
        strongest = None
        fire_direction = "NONE"

        if detections:
            # Prefer the strongest fire detection
            fire_detections = [d for d in detections if d["class_name"] == "FIRE"]
            if fire_detections:
                strongest = max(fire_detections, key=lambda d: d["confidence"])
            else:
                strongest = max(detections, key=lambda d: d["confidence"])

            fire_direction = strongest["direction"]

        return {
            "detections": detections,
            "strongest": strongest,
            "confirmed": self.fire_confirmed,
            "consecutive_count": self.consecutive_fire_frames,
            "fire_direction": fire_direction,
        }
