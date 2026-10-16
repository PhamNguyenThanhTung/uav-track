"""Small adapter from Ultralytics BoT-SORT tracks to fresh manager observations."""

from __future__ import annotations

from pathlib import Path
import time
from typing import Any, Sequence

import numpy as np
from ultralytics.trackers.bot_sort import BOTSORT
from ultralytics.utils import IterableSimpleNamespace, YAML


class _TimedEncoder:
    """Measure calls without replacing Ultralytics' configured ReID backend."""

    def __init__(self, encoder: Any, owner: "BoTSORTAdapter"):
        self._encoder = encoder
        self._owner = owner

    def __call__(self, image: np.ndarray, detections: np.ndarray):
        start = time.perf_counter()
        try:
            return self._encoder(image, detections)
        finally:
            self._owner.reid_latency_ms += (time.perf_counter() - start) * 1000.0
            self._owner.reid_calls += 1
            if self._owner.performance is not None:
                self._owner.performance['reid_end_mono_ns'] = time.perf_counter_ns()
                self._owner.performance['reid_crops'] = (
                    self._owner.performance.get('reid_crops', 0) + len(detections))

    def __getattr__(self, name: str):
        return getattr(self._encoder, name)


class BoTSORTAdapter:
    """Run the installed BoT-SORT implementation and expose measured detections only.

    BoT-SORT's Kalman boxes are useful for association, but the target manager
    must receive the original detector measurement associated with the track's
    current ``idx``. Lost/predicted tracks are intentionally omitted.
    """

    def __init__(self, tracker: BOTSORT):
        if not getattr(tracker.args, "with_reid", False) or tracker.encoder is None:
            raise RuntimeError("BoT-SORT must have its configured ReID encoder enabled")
        if tracker.args.gmc_method != "orb":
            raise RuntimeError(f"Expected ORB GMC, got {tracker.args.gmc_method!r}")

        self.tracker = tracker
        self.reid_encoder = tracker.encoder
        self.reid_calls = 0
        self.reid_latency_ms = 0.0
        self.performance = None
        self.gmc_calls = 0
        self.gmc_non_identity = 0
        self.gmc_fallbacks = 0
        self.last_gmc_non_identity = False
        self.last_gmc_warp = np.eye(2, 3, dtype=np.float32)

        self.tracker.encoder = _TimedEncoder(self.reid_encoder, self)
        self._gmc_apply = self.tracker.gmc.apply
        self.tracker.gmc.apply = self._apply_gmc

    def enable_performance_measurements(self):
        """Opt-in timings around the existing native encoder, without new work."""
        self.performance = {}
        def measured(name, function):
            def call(*args, **kwargs):
                start = time.perf_counter_ns()
                try:
                    return function(*args, **kwargs)
                finally:
                    elapsed = (time.perf_counter_ns() - start) / 1e6
                    self.performance[name] = self.performance.get(name, 0.0) + elapsed
            return call
        encoder = self.reid_encoder
        encoder._crop_detections = measured('reid_crop_ms', encoder._crop_detections)
        encoder._crops_to_tensor = measured('reid_preprocess_ms', encoder._crops_to_tensor)
        backend = encoder.model.backend
        backend.forward = measured('reid_backend_ms', backend.forward)

    @classmethod
    def from_config(cls, config_path: str | Path, device: str) -> "BoTSORTAdapter":
        """Construct BoT-SORT from the package YAML and require CUDA ONNX ReID."""
        path = Path(config_path)
        if not path.is_file():
            raise RuntimeError(f"BoT-SORT configuration does not exist: {path}")
        try:
            values = YAML.load(str(path))
            if values.get("tracker_type") != "botsort":
                raise ValueError("tracker_type must be botsort")
            if values.get("gmc_method") != "orb":
                raise ValueError("gmc_method must be orb")
            if values.get("with_reid") is not True:
                raise ValueError("with_reid must be true")
            if values.get("model") != "yolo26n-reid.onnx":
                raise ValueError("model must be yolo26n-reid.onnx")
            # Ultralytics downloads missing assets to the requested path. Keep
            # the pretrained model in the user's cache instead of the current
            # working directory (which is often the Git checkout).
            model_path = Path.home() / ".cache" / "uav_perception" / "yolo26n-reid.onnx"
            if not model_path.is_file():
                from ultralytics.utils.downloads import attempt_download_asset

                model_path.parent.mkdir(parents=True, exist_ok=True)
                attempt_download_asset(str(model_path))
            if not model_path.is_file():
                raise FileNotFoundError(f"pretrained ReID model was not downloaded to {model_path}")
            values["model"] = str(model_path)
            values["device"] = device
            tracker = BOTSORT(IterableSimpleNamespace(**values))
        except Exception as exc:
            raise RuntimeError(f"Unable to initialize BoT-SORT/ReID from {path}: {exc}") from exc

        encoder = tracker.encoder
        session = getattr(getattr(encoder, "model", None), "session", None)
        if session is None:
            raise RuntimeError(
                "Configured yolo26n-reid.onnx did not create an ONNX Runtime session"
            )
        try:
            providers = session.get_providers()
        except Exception as exc:
            raise RuntimeError(f"Unable to inspect ReID ONNX Runtime providers: {exc}") from exc
        if not providers or providers[0] != "CUDAExecutionProvider":
            raise RuntimeError(
                "ReID requires CUDAExecutionProvider as the active ONNX Runtime provider; "
                f"reported providers: {providers}"
            )
        return cls(tracker)

    def _apply_gmc(self, image: np.ndarray, detections=None) -> np.ndarray:
        started = time.perf_counter_ns() if self.performance is not None else 0
        self.gmc_calls += 1
        try:
            warp = np.asarray(self._gmc_apply(image, detections), dtype=np.float32)
            if (warp.shape != (2, 3) or not np.isfinite(warp).all()
                    or np.linalg.det(warp[:, :2].astype(np.float64)) <= 0.0):
                raise ValueError("ORB GMC returned an invalid affine transform")
        except Exception:
            self.gmc_fallbacks += 1
            self.last_gmc_non_identity = False
            self.last_gmc_warp = np.eye(2, 3, dtype=np.float32)
            raise
        finally:
            if self.performance is not None:
                ended = time.perf_counter_ns()
                self.performance.update(gmc_ms=(ended - started) / 1e6,
                                        gmc_end_mono_ns=ended)
        identity = np.eye(2, 3, dtype=np.float32)
        self.last_gmc_warp = warp.copy()
        self.last_gmc_non_identity = not np.allclose(warp, identity, rtol=1e-4, atol=1e-3)
        if self.last_gmc_non_identity:
            self.gmc_non_identity += 1
        return warp

    def update(
        self,
        boxes: Any,
        image: np.ndarray,
        detections: Sequence[Sequence[float]],
        auxiliary_features: Sequence[np.ndarray | None] | None = None,
    ) -> list[dict[str, Any]]:
        """Advance one accepted frame and adapt activated, measured tracks.

        ``detections`` is in the same person-only order as ``boxes`` and stores
        ``(x1, y1, x2, y2, confidence)`` from YOLO's original current frame.
        """
        self.reid_latency_ms = 0.0
        if self.performance is not None:
            self.performance.clear()
        self.last_gmc_warp = np.eye(2, 3, dtype=np.float32)
        self.tracker.update(boxes, img=image)
        frame_id = self.tracker.frame_id
        observations: list[dict[str, Any]] = []
        for track in self.tracker.tracked_stracks:
            if not track.is_activated or track.frame_id != frame_id:
                continue
            try:
                index = int(track.idx)
            except (TypeError, ValueError, OverflowError):
                continue
            if index < 0 or index >= len(detections):
                continue

            measured = detections[index]
            feature = getattr(track, "curr_feat", None)
            auxiliary = (
                auxiliary_features[index]
                if auxiliary_features is not None and index < len(auxiliary_features)
                else None
            )
            observations.append({
                "track_id": str(int(track.track_id)),
                "bbox": [float(v) for v in measured[:4]],
                "confidence": float(measured[4]),
                "feature": None if feature is None else np.asarray(feature, dtype=np.float32).copy(),
                "appearance_aux": auxiliary,
                "mean": None if track.mean is None else np.asarray(track.mean).copy(),
                "time_since_update": 0,
            })
        return observations

    def reset(self) -> None:
        self.tracker.reset()
