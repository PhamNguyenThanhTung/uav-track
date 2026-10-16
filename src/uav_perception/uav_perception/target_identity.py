#!/usr/bin/env python3
"""Guard the selected logical target against tracker-ID swaps."""

from collections import deque
from dataclasses import dataclass
from enum import Enum
import math
from typing import Any, Deque, List, Optional, Sequence, Tuple

import numpy as np


class TrackingStatus(str, Enum):
    SEARCHING = "SEARCHING"
    TRACKING = "TRACKING"
    UNCERTAIN = "UNCERTAIN"
    PREDICTING = "PREDICTING"
    # Keep the old public name as an alias for callers that used COASTING.
    COASTING = "PREDICTING"
    LOST = "LOST"


@dataclass
class TargetState:
    target_id: str
    tracker_id: Optional[str] = None
    class_name: str = "person"
    confidence: float = 0.0
    bbox: Optional[List[float]] = None
    status: str = TrackingStatus.SEARCHING
    last_seen_time: float = 0.0
    id_switch_count: int = 0
    lost_count: int = 0

    @property
    def center(self) -> Optional[Tuple[float, float]]:
        if self.bbox is None or len(self.bbox) < 4:
            return None
        return ((self.bbox[0] + self.bbox[2]) / 2.0, (self.bbox[1] + self.bbox[3]) / 2.0)

    @property
    def is_active(self) -> bool:
        return self.status == TrackingStatus.TRACKING and self.bbox is not None


@dataclass
class _Candidate:
    track_id: str
    bbox: List[float]
    confidence: float
    feature: Optional[np.ndarray]
    clean_crop: bool = True
    ambiguous_crop: bool = False
    appearance: float = -1.0
    motion_error: float = math.inf
    motion_limit: float = 0.0
    motion_valid: bool = False
    track_velocity: Optional[Tuple[float, float]] = None
    auxiliary_feature: Optional[np.ndarray] = None
    auxiliary_similarity: float = -1.0
    motion_reason: str = "not_evaluated"
    extent_ratio: Optional[float] = None
    truncated_crop: bool = False


class TargetIdentityManager:
    """Associate detections to one selected person using appearance and motion."""

    def __init__(
        self,
        coasting_timeout_sec: float = 1.5,
        default_target_id: str = "1",
        class_name: str = "person",
        max_reacquisition_distance_px: float = 130.0,
        max_appearance_references: int = 8,
        min_appearance_similarity: float = 0.76,
        min_appearance_margin: float = 0.05,
        max_crop_overlap_fraction: float = 0.18,
        max_association_overlap_fraction: float = 0.20,
        min_auxiliary_similarity: float = 0.75,
        debug_enabled: bool = False,
        reacquisition_confirmation_frames: int = 3,
        reacquisition_min_confidence: float = 0.60,
        reacquisition_min_auxiliary_similarity: float = 0.90,
    ):
        self.coasting_timeout_sec = float(coasting_timeout_sec)
        self.default_target_id = str(default_target_id)
        self.class_name = str(class_name)
        # Legacy parameter bounds the base jitter margin, not the whole motion
        # envelope. Position uncertainty grows with source dt and body scale.
        self.max_reacquisition_distance_px = float(max_reacquisition_distance_px)
        self.max_appearance_references = max(1, int(max_appearance_references))
        self.min_appearance_similarity = float(min_appearance_similarity)
        self.min_appearance_margin = float(min_appearance_margin)
        self.max_crop_overlap_fraction = float(max_crop_overlap_fraction)
        self.max_association_overlap_fraction = float(max_association_overlap_fraction)
        self.min_auxiliary_similarity = float(min_auxiliary_similarity)
        self.debug_enabled = bool(debug_enabled)
        self.last_debug = None
        self.reacquisition_confirmation_frames = max(2, int(reacquisition_confirmation_frames))
        self.reacquisition_min_confidence = float(reacquisition_min_confidence)
        self.reacquisition_min_auxiliary_similarity = float(reacquisition_min_auxiliary_similarity)
        self._session_invalidated = False
        self._pending_reacquisition_box = None
        self._pending_reacquisition_time = None
        self._pending_reacquisition_features = []
        self._pending_reacquisition_started = None
        self._frame_size = None
        self._recovery_motion_warmup = False

        self._state = TargetState(
            target_id=self.default_target_id,
            class_name=self.class_name,
            status=TrackingStatus.SEARCHING,
        )
        self._pending_switch_event: Optional[Tuple[str, str]] = None
        self._has_seen_target = False
        self._references: Deque[np.ndarray] = deque(maxlen=self.max_appearance_references - 1)
        self._anchor_feature: Optional[np.ndarray] = None
        self._auxiliary_anchor: Optional[np.ndarray] = None
        self._history: Deque[Tuple[float, List[float]]] = deque(maxlen=6)
        self._pending_tracker_id: Optional[str] = None
        self._pending_switch_count = 0
        self._overlap_unresolved = False
        self.abstention_count = 0
        self.accepted_observation_count = 0
        self._trusted_confidence = 0.0
        self._last_update_time: Optional[float] = None
        self._previous_detection_count: Optional[int] = None

    @property
    def state(self) -> TargetState:
        return self._state

    @property
    def active_target_id(self) -> str:
        return self._state.target_id

    @property
    def active_tracker_id(self) -> Optional[str]:
        return self._state.tracker_id

    @property
    def tracking_status(self) -> str:
        return self._state.status

    @property
    def tracked_bbox(self) -> Optional[List[float]]:
        return self._state.bbox

    @property
    def target_conf(self) -> float:
        return self._state.confidence

    @property
    def id_switch_count(self) -> int:
        """Number of validated raw tracker-ID changes for this logical target."""
        return self._state.id_switch_count

    @property
    def raw_tid_switch_count(self) -> int:
        return self._state.id_switch_count

    @property
    def lost_track_count(self) -> int:
        return self._state.lost_count

    @property
    def last_track_seen_time(self) -> float:
        return self._state.last_seen_time

    @property
    def appearance_reference_count(self) -> int:
        return len(self._references) + int(self._anchor_feature is not None)

    def pop_id_switch_event(self) -> Optional[Tuple[str, str]]:
        event = self._pending_switch_event
        self._pending_switch_event = None
        return event

    def update(self, active_tracks: Sequence[Any], current_time: float,
               detection_bboxes: Optional[Sequence[Sequence[float]]] = None,
               camera_warp: Optional[np.ndarray] = None,
               image_size: Optional[Tuple[int, int]] = None) -> TargetState:
        if not self.debug_enabled:
            return self._update(active_tracks, current_time, detection_bboxes, camera_warp, image_size)
        before = dict(status=self._state.status, tracker_id=self._state.tracker_id,
                      last_seen_time=self._state.last_seen_time,
                      reference_count=self.appearance_reference_count,
                      anchor=None if self._anchor_feature is None else self._anchor_feature.tolist(),
                      history=list(self._history), overlap_unresolved=self._overlap_unresolved)
        self._debug_candidates = []
        self._decision_reason = "not_evaluated"
        state = self._update(active_tracks, current_time, detection_bboxes, camera_warp, image_size)
        self.last_debug = dict(
            source_time=float(current_time), before=before,
            after=dict(status=state.status, tracker_id=state.tracker_id, bbox=state.bbox,
                       last_seen_time=state.last_seen_time, reference_count=self.appearance_reference_count),
            reason=self._decision_reason,
            candidates=[dict(track_id=c.track_id, bbox=c.bbox, confidence=c.confidence,
                             appearance=c.appearance, auxiliary_similarity=c.auxiliary_similarity,
                             clean_crop=c.clean_crop, ambiguous_crop=c.ambiguous_crop,
                             motion_valid=c.motion_valid, motion_error=c.motion_error,
                             motion_limit=c.motion_limit, motion_reason=c.motion_reason,
                             extent_ratio=c.extent_ratio, truncated_crop=c.truncated_crop)
                        for c in self._debug_candidates])
        return state

    def mark_stream_stale(self, current_time: float,
                          camera_warp: Optional[np.ndarray] = None) -> TargetState:
        """Age identity without consuming an image timestamp or learning a target.

        A completed but expired tracker frame still advances the camera frame
        of the motion history. Timer-only watchdog calls leave it unchanged.
        """
        last_source_time = self._last_update_time
        try:
            return self.update([], current_time, camera_warp=camera_warp)
        finally:
            self._last_update_time = last_source_time

    def _update(self, active_tracks: Sequence[Any], current_time: float,
                detection_bboxes: Optional[Sequence[Sequence[float]]] = None,
                camera_warp: Optional[np.ndarray] = None,
                image_size: Optional[Tuple[int, int]] = None) -> TargetState:
        """Update from fresh tracker measurements and the image source timestamp.

        A selected TID is never sufficient to accept an observation. The first
        fresh track initializes the target; subsequent observations need a valid
        appearance feature and a plausible short-term motion step. Ambiguity emits
        no bbox, so neither the gimbal nor the target detection output can follow
        an unverified person. When provided, ``camera_warp`` maps the previous
        image into the current image before the motion history is evaluated.
        """
        if self._session_invalidated:
            self._decision_reason = "invalidated_identity_session"
            self._state.bbox = None
            return self._state

        now = float(current_time)
        if not math.isfinite(now):
            self._decision_reason = "invalid_time"
            self._mark_lost(invalidate=True)
            return self._state
        if self._last_update_time is not None and now <= self._last_update_time:
            self._decision_reason = "nonmonotonic_time"
            self._pending_tracker_id = None
            self._pending_switch_count = 0
            if self._has_seen_target:
                if now < self._last_update_time:
                    self._mark_lost(invalidate=True)
                else:
                    self._abstain(now, uncertain=True)
            return self._state
        self._last_update_time = now
        self._frame_size = image_size
        if camera_warp is not None:
            self._transform_motion_history(camera_warp)

        fresh = [track for track in active_tracks if self.is_fresh_observation(track)]
        candidates = [candidate for track in fresh if (candidate := self._candidate(track)) is not None]
        self._debug_candidates = candidates
        self._mark_crop_quality(candidates, detection_bboxes)
        detection_count = len(detection_bboxes) if detection_bboxes is not None else len(candidates)
        count_dropped = (self._previous_detection_count is not None
                         and detection_count < self._previous_detection_count)
        self._previous_detection_count = detection_count

        if not self._has_seen_target:
            if not candidates:
                self._decision_reason = "no_initial_candidate"
                self._state.status = TrackingStatus.SEARCHING
                self._state.bbox = None
                return self._state
            # Preserve the existing initial-selection rule. Identity protection
            # begins immediately after the first fresh observation.
            first = candidates[0]
            if not first.clean_crop or first.feature is None:
                self._decision_reason = "initial_crop_or_feature"
                self._state.status = TrackingStatus.SEARCHING
                self._state.bbox = None
                return self._state
            self._accept(first, now)
            self._decision_reason = "initial_selection"
            return self._state

        if (now - self._state.last_seen_time >= self.coasting_timeout_sec
                and self._state.status != TrackingStatus.LOST):
            self._mark_lost()

        for candidate in candidates:
            if candidate.feature is not None:
                candidate.appearance = self._appearance_similarity(candidate.feature)
            candidate.auxiliary_similarity = self._auxiliary_similarity(candidate.auxiliary_feature)
            plausible, error, limit = self._motion_consistent(candidate, now)
            candidate.motion_valid = plausible
            candidate.motion_error = error
            candidate.motion_limit = limit

        if self._state.status == TrackingStatus.LOST:
            return self._evaluate_reacquisition(candidates, now)

        compatible = [c for c in candidates if c.feature is not None
                      and c.appearance >= self.min_appearance_similarity
                      and c.auxiliary_similarity >= self.min_auxiliary_similarity]
        # A disappearing nearby detection can leave a contaminated target box
        # that looks separated only because the other person is no longer
        # detected. Appearance alone must not confirm that transition. Clean
        # target crops remain usable when someone elsewhere leaves the image.
        crowded_count_drop = count_dropped and any(
            not c.clean_crop and (c.motion_valid or c.track_id == self._state.tracker_id)
            for c in compatible
        )
        if crowded_count_drop or any(c.ambiguous_crop and (c.motion_valid or c.track_id == self._state.tracker_id)
               for c in compatible) or (not compatible and any(
                   c.ambiguous_crop and c.track_id == self._state.tracker_id for c in candidates)):
            self._overlap_unresolved = True
            self._decision_reason = "crowded_or_ambiguous_target_crop"
            # Do not let a neighboring, apparently clean box win simply because
            # the expected target's mixed crop has been excluded from ranking.
            self._pending_tracker_id = None
            self._pending_switch_count = 0
            self._abstain(now, uncertain=True)
            return self._state
        # A single box immediately after a multi-person overlap may be a merged
        # detector crop. Its lack of another box is not evidence of separation.
        if self._overlap_unresolved and len(candidates) < 2:
            self._decision_reason = "unresolved_overlap_single_candidate"
            self._pending_tracker_id = None
            self._pending_switch_count = 0
            self._abstain(now, uncertain=bool(candidates))
            return self._state

        viable = [
            candidate for candidate in candidates
            if not candidate.ambiguous_crop
            and (not self._overlap_unresolved or candidate.clean_crop or (
                self._auxiliary_anchor is not None and candidate.auxiliary_similarity >= 0.90
            ))
            and candidate.feature is not None
            and candidate.motion_valid
            and candidate.appearance >= self.min_appearance_similarity
            and candidate.auxiliary_similarity >= self.min_auxiliary_similarity
            and candidate.motion_error <= candidate.motion_limit
        ]
        if not viable:
            # Persistent geometry-only interruption may recover within the
            # coasting window, retaining a coarse position envelope instead of
            # the unreliable extrapolated vertical/heading hard gates.
            if candidates and now-self._state.last_seen_time >= .5:
                return self._evaluate_reacquisition(candidates, now, require_spatial=True)
            self._decision_reason = "no_candidate_passes_gates"
            self._pending_tracker_id = None
            self._pending_switch_count = 0
            self._abstain(now, uncertain=bool(candidates))
            return self._state

        best = self._select_candidate(viable)
        if best is None:
            self._decision_reason = "competing_candidates"
            self._pending_tracker_id = None
            self._pending_switch_count = 0
            self._abstain(now, uncertain=True)
            return self._state

        old_tid = self._state.tracker_id
        if old_tid is not None and (
            best.track_id != old_tid or self._overlap_unresolved
            or self._state.status == TrackingStatus.UNCERTAIN
        ):
            # A tracker-ID reassignment must persist across two fresh frames.
            # This limits one-frame association glitches without tracking a
            # second identity or predicting a bbox to the controller.
            if self._pending_tracker_id == best.track_id:
                self._pending_switch_count += 1
            else:
                self._pending_tracker_id = best.track_id
                self._pending_switch_count = 1
            if self._pending_switch_count < 2:
                self._decision_reason = "pending_two_frame_confirmation"
                self._abstain(now, uncertain=True)
                return self._state
        else:
            self._pending_tracker_id = None
            self._pending_switch_count = 0

        self._accept(best, now)
        self._decision_reason = "accepted_continuation_or_reassociation"
        return self._state

    def _evaluate_reacquisition(self, candidates: Sequence[_Candidate], now: float,
                               require_spatial: bool = False) -> TargetState:
        """Recover the enrolled identity, never select a new identity after loss.

        The old trajectory cannot veto observations indefinitely. Identity,
        crop quality, competition, and a fresh stable confirmation sequence
        remain mandatory after its coasting uncertainty window expires.
        """
        if self._state.status != TrackingStatus.LOST:
            self._abstain(now, uncertain=bool(candidates))
        eligible = [c for c in candidates if c.clean_crop and not c.ambiguous_crop
                  and c.feature is not None
                  and c.confidence >= self.reacquisition_min_confidence
                  and (not require_spatial or c.motion_error <= c.motion_limit)
                  and (self._auxiliary_anchor is None or
                       c.auxiliary_similarity >= self.reacquisition_min_auxiliary_similarity)]
        ranked = sorted(eligible, key=lambda c: c.appearance + .25*c.auxiliary_similarity, reverse=True)
        minimum = (self.min_appearance_similarity if self._auxiliary_anchor is not None
                   else max(.90, self.min_appearance_similarity))
        ambiguous = (len(ranked) > 1 and
                (ranked[0].appearance + .25*ranked[0].auxiliary_similarity
                 - ranked[1].appearance - .25*ranked[1].auxiliary_similarity)
                < self.min_appearance_margin)
        if not ranked or ranked[0].appearance < minimum or ambiguous:
            self._decision_reason = 'reacquisition_ambiguous' if ambiguous else 'reacquisition_gates'
            if ambiguous or not self._hold_reacquisition_evidence(candidates, now, minimum):
                self._clear_reacquisition_pending()
            return self._state
        best = ranked[0]
        stable = False
        if self._pending_tracker_id == best.track_id and self._pending_reacquisition_box is not None:
            dt = now - self._pending_reacquisition_time
            step = math.dist(self._center(best.bbox), self._center(self._pending_reacquisition_box))
            height = best.bbox[3]-best.bbox[1]
            stable = (0 < dt <= .75 and step <= max(24.0, .25*height + .75*height*dt)
                      and self._pending_reacquisition_started is not None
                      and now-self._pending_reacquisition_started <= self.coasting_timeout_sec)
        self._pending_switch_count = self._pending_switch_count+1 if stable else 1
        if not stable:
            self._pending_reacquisition_features = []
            self._pending_reacquisition_started = now
        self._pending_reacquisition_features.append(best.feature.copy())
        self._pending_tracker_id = best.track_id
        self._pending_reacquisition_box = list(best.bbox)
        self._pending_reacquisition_time = now
        if self._pending_switch_count < self.reacquisition_confirmation_frames:
            self._decision_reason = 'pending_reacquisition_confirmation'
            return self._state
        confirmed_feature = self._unit_feature(np.mean(self._pending_reacquisition_features, axis=0))
        self._history.clear()
        self._recovery_motion_warmup = True
        self._accept(best, now, update_reference=False)
        # One bounded descriptor from the confirmed clean sequence, rather
        # than replacing the enrollment anchor with a single recovery crop.
        if confirmed_feature is not None:
            self._references.append(confirmed_feature)
        self._clear_reacquisition_pending()
        self._decision_reason = 'reacquired_enrolled_identity'
        return self._state

    def _hold_reacquisition_evidence(self, candidates: Sequence[_Candidate], now: float,
                                     minimum_appearance: float) -> bool:
        """Retain clean positives across uncounted bad crops for a bounded time.

        A continuous fresh same-ID track and independent appearance cue may
        bridge pose/overlap noise. Such frames never count as positive evidence,
        teach references, or emit a target bbox. Competing matches reset above.
        """
        if (self._pending_reacquisition_started is None or
                now-self._pending_reacquisition_started > self.coasting_timeout_sec):
            return False
        candidate = next((c for c in candidates if c.track_id == self._pending_tracker_id), None)
        if candidate is None or candidate.feature is None or candidate.confidence < self.reacquisition_min_confidence:
            return False
        if self._auxiliary_anchor is not None:
            if candidate.auxiliary_similarity < self.reacquisition_min_auxiliary_similarity:
                return False
        elif candidate.appearance < minimum_appearance:
            return False
        dt = now-self._pending_reacquisition_time
        height = candidate.bbox[3]-candidate.bbox[1]
        step = math.dist(self._center(candidate.bbox), self._center(self._pending_reacquisition_box))
        if not 0 < dt <= .75 or step > max(24.0, .25*height + .75*height*dt):
            return False
        self._pending_reacquisition_box = list(candidate.bbox)
        self._pending_reacquisition_time = now
        return True

    def _clear_reacquisition_pending(self) -> None:
        self._pending_tracker_id = None
        self._pending_switch_count = 0
        self._pending_reacquisition_box = None
        self._pending_reacquisition_time = None
        self._pending_reacquisition_features = []
        self._pending_reacquisition_started = None

    def _transform_motion_history(self, camera_warp: np.ndarray) -> None:
        """Move stored image-space boxes into the current camera frame.

        BoT-SORT estimates the previous-to-current affine camera transform for
        GMC. Applying it before the motion gate prevents camera movement from
        looking like an impossible target jump.
        """
        try:
            warp = np.asarray(camera_warp, dtype=np.float32)
        except (TypeError, ValueError):
            return
        if warp.shape != (2, 3) or not np.isfinite(warp).all() or not self._history:
            return
        transformed = deque(maxlen=self._history.maxlen)
        for timestamp, bbox in self._history:
            x1, y1, x2, y2 = bbox
            corners = np.asarray(
                [[x1, y1], [x2, y1], [x2, y2], [x1, y2]], dtype=np.float32
            )
            points = corners @ warp[:, :2].T + warp[:, 2]
            transformed.append((timestamp, [
                float(points[:, 0].min()), float(points[:, 1].min()),
                float(points[:, 0].max()), float(points[:, 1].max()),
            ]))
        self._history = transformed

    def _candidate(self, track: Any) -> Optional[_Candidate]:
        track_id = self._extract_track_id(track)
        if track_id is None:
            return None
        try:
            bbox = [float(value) for value in self._extract_ltrb(track)]
        except (TypeError, ValueError):
            return None
        if len(bbox) != 4 or not all(math.isfinite(value) for value in bbox):
            return None
        if bbox[2] <= bbox[0] or bbox[3] <= bbox[1]:
            return None
        return _Candidate(
            track_id=track_id,
            bbox=bbox,
            confidence=self._extract_conf(track),
            feature=self._extract_feature(track),
            track_velocity=self._extract_track_velocity(track),
            auxiliary_feature=self._extract_auxiliary_feature(track),
        )

    @staticmethod
    def _extract_track_velocity(track: Any) -> Optional[Tuple[float, float]]:
        # BoT-SORT stores vx/vy per Kalman frame step. Only their direction is
        # reused here; treating these values as px/second would be incorrect.
        raw = track.get("mean") if isinstance(track, dict) else getattr(track, "mean", None)
        if raw is None:
            return None
        try:
            vx, vy = float(raw[4]), float(raw[5])
        except (TypeError, ValueError, IndexError):
            return None
        return (vx, vy) if math.isfinite(vx) and math.isfinite(vy) else None

    def _mark_crop_quality(self, candidates: Sequence[_Candidate],
                           detection_bboxes: Optional[Sequence[Sequence[float]]] = None) -> None:
        boxes = [c.bbox for c in candidates] if detection_bboxes is None else list(detection_bboxes)
        for i, candidate in enumerate(candidates):
            max_intersection_fraction = 0.0
            max_torso_fraction = 0.0
            area = self._area(candidate.bbox)
            torso = self._torso_box(candidate.bbox)
            torso_area = self._area(torso)
            own = i if detection_bboxes is None else None
            if detection_bboxes is not None and boxes:
                overlaps = []
                for box in boxes:
                    intersection = self._intersection(candidate.bbox, box)
                    union = area + self._area(box) - intersection
                    overlaps.append(intersection / union if union > 0.0 else 0.0)
                nearest = max(range(len(boxes)), key=lambda j: overlaps[j])
                if overlaps[nearest] > 0.90:
                    own = nearest
            for j, box in enumerate(boxes):
                if own == j:
                    continue
                intersection = self._intersection(candidate.bbox, box)
                if area > 0.0:
                    max_intersection_fraction = max(max_intersection_fraction, intersection / area)
                if torso_area > 0.0:
                    max_torso_fraction = max(max_torso_fraction,
                        self._intersection(torso, self._torso_box(box)) / torso_area)
            candidate.clean_crop = max_intersection_fraction <= self.max_crop_overlap_fraction
            candidate.ambiguous_crop = max_torso_fraction > self.max_association_overlap_fraction
            if self._frame_size is not None:
                width, height = self._frame_size
                margin = .005*min(width, height)
                x1, y1, x2, y2 = candidate.bbox
                candidate.truncated_crop = (x1 <= margin or y1 <= margin or
                                            x2 >= width-margin or y2 >= height-margin)
                if candidate.truncated_crop:
                    candidate.clean_crop = False

    @staticmethod
    def _torso_box(box: Sequence[float]) -> List[float]:
        x1, y1, x2, y2 = box
        w, h = x2 - x1, y2 - y1
        # Match the existing auxiliary descriptor region. Overlapping limbs can
        # dirty a reference without making separated torsos the same person.
        return [x1 + .2*w, y1 + .2*h, x1 + .8*w, y1 + .6*h]

    def _motion_consistent(self, candidate: _Candidate, now: float) -> Tuple[bool, float, float]:
        candidate.motion_reason = "ok"
        if not self._history:
            return True, 0.0, math.inf
        previous_time, previous_bbox = self._history[-1]
        dt = now - previous_time
        if dt <= 0.0:
            candidate.motion_reason = "nonpositive_motion_dt"
            return False, math.inf, 0.0
        previous_center = self._center(previous_bbox)
        candidate_center = self._center(candidate.bbox)
        prediction = previous_center
        previous_velocity = (0.0, 0.0)
        previous_speed = 0.0
        height = max(previous_bbox[3] - previous_bbox[1], candidate.bbox[3] - candidate.bbox[1])
        if (len(self._history) >= 2 and (not self._recovery_motion_warmup or
                (len(self._history) >= 3 and previous_time-self._history[0][0] >= .2-1e-9))):
            # Fit the existing short clean history instead of amplifying the
            # centroid jitter of a single walking-pose step.
            ts = np.asarray([t - previous_time for t, _ in self._history])
            centers = np.asarray([self._center(b) for _, b in self._history])
            centered_ts = ts - ts.mean()
            denominator = float(np.dot(centered_ts, centered_ts))
            if denominator > 1e-9:
                velocity = np.sum(centered_ts[:, None] * centers, axis=0) / denominator
                # Small centroid oscillations caused by walking pose are not a
                # reliable extrapolation direction across an occlusion.
                velocity[np.abs(velocity) < 0.10 * height] = 0.0
                previous_speed = float(np.linalg.norm(velocity))
                speed_limit = 0.75 * height
                if previous_speed > speed_limit:
                    velocity *= speed_limit / previous_speed
                previous_velocity = (float(velocity[0]), float(velocity[1]))
                previous_speed = math.hypot(*previous_velocity)
                prediction = (
                    previous_center[0] + previous_velocity[0] * dt,
                    previous_center[1] + previous_velocity[1] * dt,
                )
        residual = math.hypot(candidate_center[0] - prediction[0], candidate_center[1] - prediction[1])
        limit = max(
            24.0, min(self.max_reacquisition_distance_px, 0.15 * height)
            + min(0.75 * height, 0.5 * height * dt * dt)
            + 0.25 * previous_speed * dt,
        )
        # A horizontal crossing must not snap onto a person on a different
        # vertical path merely because their appearance embedding is similar.
        vertical_limit = max(20.0, 0.10 * height + 0.15 * abs(previous_velocity[1]) * dt)
        if abs(candidate_center[1] - prediction[1]) > vertical_limit:
            candidate.motion_reason = "vertical_path"
            return False, residual, limit

        if (
            previous_speed > 0.10 * height and candidate.track_velocity is not None
            and (self._overlap_unresolved or candidate.track_id != self._state.tracker_id)
        ):
            vx, vy = candidate.track_velocity
            candidate_speed = math.hypot(vx, vy)
            if candidate_speed > 0.002 * height:
                heading_cosine = (vx * previous_velocity[0] + vy * previous_velocity[1]) / (
                    candidate_speed * previous_speed
                )
                if heading_cosine < -0.25:
                    local_turn = (
                        candidate.track_id == self._state.tracker_id
                        and candidate.clean_crop and dt <= 0.5
                        and (candidate.appearance >= 0.92 or (
                            self._auxiliary_anchor is not None and candidate.auxiliary_similarity >= 0.90
                        ))
                        and math.hypot(candidate_center[0] - previous_center[0],
                                       candidate_center[1] - previous_center[1]) <= 0.12 * height
                    )
                    if not local_turn:
                        candidate.motion_reason = "opposite_tracker_heading"
                        return False, residual, limit

        step_x = candidate_center[0] - previous_center[0]
        step_y = candidate_center[1] - previous_center[1]
        step = math.hypot(step_x, step_y)
        if previous_speed > 45.0 and step > 0.22 * height:
            dot = step_x * previous_velocity[0] + step_y * previous_velocity[1]
            direction_cosine = dot / max(step * previous_speed, 1e-6)
            if direction_cosine < -0.70:
                candidate.motion_reason = "opposite_displacement"
                return False, residual, limit

        old_height = previous_bbox[3] - previous_bbox[1]
        new_height = candidate.bbox[3] - candidate.bbox[1]
        if old_height > 0.0 and new_height > 0.0:
            # A walking person's arm/leg pose changes box width dramatically.
            # Height is the more stable scale cue; area rejected genuine narrow
            # poses and left the old trajectory frozen until timeout.
            ratio = new_height / old_height
            candidate.extent_ratio = ratio
            if ratio < 0.65 or ratio > 1.55:
                candidate.motion_reason = "height_ratio"
                return False, residual, limit
        if residual > limit:
            candidate.motion_reason = "motion_residual"
        return residual <= limit, residual, limit

    def _appearance_similarity(self, feature: np.ndarray) -> float:
        refs = list(self._references)
        if self._anchor_feature is not None:
            refs.append(self._anchor_feature)
        return max((float(np.dot(reference, feature)) for reference in refs
                    if reference.shape == feature.shape), default=-1.0)

    def _auxiliary_similarity(self, feature: Optional[np.ndarray]) -> float:
        if self._auxiliary_anchor is None:
            return 1.0
        if feature is None or feature.shape != self._auxiliary_anchor.shape:
            return -1.0
        return float(np.dot(self._auxiliary_anchor, feature))

    def _select_candidate(self, viable: Sequence[_Candidate]) -> Optional[_Candidate]:
        """Rank already validated candidates and reject a close score tie."""
        if not viable:
            return None
        score = lambda candidate: (
            candidate.appearance - 0.12 * candidate.motion_error / max(candidate.motion_limit, 1.0)
        )
        ranked = sorted(viable, key=lambda candidate: (score(candidate), candidate.confidence), reverse=True)
        if len(ranked) > 1 and score(ranked[0]) - score(ranked[1]) < self.min_appearance_margin:
            return None
        return ranked[0]

    def _accept(self, candidate: _Candidate, now: float, update_reference: bool = True) -> None:
        old_tid = self._state.tracker_id
        if old_tid is not None and candidate.track_id != old_tid:
            self._state.id_switch_count += 1
            self._pending_switch_event = (old_tid, candidate.track_id)
        self._state.tracker_id = candidate.track_id
        self._state.bbox = list(candidate.bbox)
        self._state.confidence = float(candidate.confidence)
        self._trusted_confidence = float(candidate.confidence)
        self._state.last_seen_time = now
        self._state.status = TrackingStatus.TRACKING
        self._has_seen_target = True
        if old_tid is None and candidate.clean_crop:
            self._anchor_feature = candidate.feature.copy() if candidate.feature is not None else None
            self._auxiliary_anchor = (
                candidate.auxiliary_feature.copy() if candidate.auxiliary_feature is not None else None
            )
        self.accepted_observation_count += 1
        geometry_is_trusted = True
        if self._history:
            previous_bbox = self._history[-1][1]
            top_step = candidate.bbox[1] - previous_bbox[1]
            bottom_step = candidate.bbox[3] - previous_bbox[3]
            height = previous_bbox[3] - previous_bbox[1]
            # Even in a separated crop, an edge disappearing changes the measured
            # centroid without moving the person. Allow small detector jitter
            # and coherent translation, but do not learn velocity from a
            # one-sided contraction/expansion of the body box.
            extent_noise = 0.02 * height + 2.0 * min(abs(top_step), abs(bottom_step))
            geometry_is_trusted = abs(top_step - bottom_step) <= extent_noise
        if not candidate.ambiguous_crop and geometry_is_trusted:
            self._history.append((now, list(candidate.bbox)))
            while len(self._history) > 2 and now - self._history[0][0] > 1.0:
                self._history.popleft()
            if len(self._history) >= 3 and now-self._history[0][0] >= .2-1e-9:
                self._recovery_motion_warmup = False
        self._overlap_unresolved = False
        self._pending_tracker_id = None
        self._pending_switch_count = 0
        stable_tracker_assignment = old_tid is None or candidate.track_id == old_tid
        motion_is_confident = old_tid is None or (
            candidate.motion_limit > 0.0
            and candidate.motion_error <= 0.5 * candidate.motion_limit
        )
        appearance_is_confident = old_tid is None or candidate.appearance >= 0.85
        if (
            old_tid is not None
            and update_reference
            and candidate.clean_crop
            and geometry_is_trusted
            and candidate.confidence >= self.reacquisition_min_confidence
            and candidate.feature is not None
            and stable_tracker_assignment
            and motion_is_confident
            and appearance_is_confident
        ):
            if not self._references or self._appearance_similarity(candidate.feature) >= 0.70:
                self._references.append(candidate.feature.copy())

    def _abstain(self, now: float, uncertain: bool) -> None:
        self.abstention_count += 1
        self._state.bbox = None
        elapsed = now - self._state.last_seen_time
        self._state.confidence = self._trusted_confidence * max(
            0.0, min(1.0, 1.0 - elapsed / max(self.coasting_timeout_sec, 1e-6))
        )
        if elapsed >= self.coasting_timeout_sec:
            self._mark_lost()
        else:
            self._state.status = TrackingStatus.UNCERTAIN if uncertain else TrackingStatus.PREDICTING

    def _mark_lost(self, invalidate: bool = False) -> None:
        if self._state.status != TrackingStatus.LOST:
            self._state.lost_count += 1
        self._state.bbox = None
        self._state.confidence = 0.0
        self._state.status = TrackingStatus.LOST
        self._previous_detection_count = None
        self._clear_reacquisition_pending()
        if invalidate:
            self._session_invalidated = True
            self._references.clear()
            self._anchor_feature = None
            self._auxiliary_anchor = None
            self._history.clear()
            self._overlap_unresolved = False

    def reset(self) -> None:
        self._session_invalidated = False
        self._recovery_motion_warmup = False
        self._clear_reacquisition_pending()
        self._state.tracker_id = None
        self._state.confidence = 0.0
        self._state.bbox = None
        self._state.status = TrackingStatus.SEARCHING
        self._state.last_seen_time = 0.0
        self._state.id_switch_count = 0
        self._state.lost_count = 0
        self._pending_switch_event = None
        self._has_seen_target = False
        self._references.clear()
        self._anchor_feature = None
        self._auxiliary_anchor = None
        self._history.clear()
        self._pending_tracker_id = None
        self._pending_switch_count = 0
        self.abstention_count = 0
        self.accepted_observation_count = 0
        self._overlap_unresolved = False
        self._trusted_confidence = 0.0
        self._last_update_time = None
        self._previous_detection_count = None

    @staticmethod
    def is_fresh_observation(track: Any) -> bool:
        if isinstance(track, dict):
            return track.get("time_since_update", 0) == 0
        return getattr(track, "time_since_update", 0) == 0

    @staticmethod
    def _extract_track_id(track: Any) -> Optional[str]:
        if hasattr(track, "track_id"):
            return str(track.track_id)
        if isinstance(track, dict) and "track_id" in track:
            return str(track["track_id"])
        return None

    @staticmethod
    def _extract_ltrb(track: Any) -> Sequence[float]:
        if hasattr(track, "to_ltrb"):
            try:
                # Use the detection measurement matching get_feature(), not the
                # Kalman-predicted box that may have crossed onto another person.
                box = track.to_ltrb(orig=True, orig_strict=True)
            except TypeError:
                box = track.to_ltrb()
            if box is not None:
                return box
        if hasattr(track, "bbox"):
            return track.bbox
        if isinstance(track, dict) and "bbox" in track:
            return track["bbox"]
        if isinstance(track, (list, tuple)) and len(track) >= 4:
            return track[:4]
        raise ValueError(f"Unable to extract original detection box from track: {track}")

    @staticmethod
    def _extract_feature(track: Any) -> Optional[np.ndarray]:
        raw = None
        getter = getattr(track, "get_feature", None)
        if callable(getter):
            raw = getter()
        elif isinstance(track, dict):
            raw = track.get("feature")
        return TargetIdentityManager._unit_feature(raw)

    @staticmethod
    def _extract_auxiliary_feature(track: Any) -> Optional[np.ndarray]:
        getter = getattr(track, "get_det_supplementary", None)
        supplementary = getter() if callable(getter) else track if isinstance(track, dict) else None
        raw = supplementary.get("appearance_aux") if isinstance(supplementary, dict) else None
        return TargetIdentityManager._unit_feature(raw)

    @staticmethod
    def _unit_feature(raw: Any) -> Optional[np.ndarray]:
        if raw is None:
            return None
        try:
            feature = np.asarray(raw, dtype=np.float32).reshape(-1)
        except (TypeError, ValueError):
            return None
        if feature.size == 0 or not np.all(np.isfinite(feature)):
            return None
        norm = float(np.linalg.norm(feature))
        if not math.isfinite(norm) or norm <= 1e-12:
            return None
        return feature / norm

    @staticmethod
    def _extract_conf(track: Any) -> float:
        getter = getattr(track, "get_det_conf", None)
        if callable(getter):
            confidence = getter()
            if confidence is not None:
                return float(confidence)
        if hasattr(track, "confidence"):
            return float(track.confidence)
        if isinstance(track, dict) and "confidence" in track:
            return float(track["confidence"])
        return 0.50

    @staticmethod
    def _center(box: Sequence[float]) -> Tuple[float, float]:
        return ((box[0] + box[2]) * 0.5, (box[1] + box[3]) * 0.5)

    @staticmethod
    def _area(box: Sequence[float]) -> float:
        return max(0.0, box[2] - box[0]) * max(0.0, box[3] - box[1])

    @staticmethod
    def _intersection(a: Sequence[float], b: Sequence[float]) -> float:
        return (
            max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
            * max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
        )
