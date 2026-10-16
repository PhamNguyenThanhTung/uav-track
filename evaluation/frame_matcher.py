"""Source-stamp matcher for camera, raw detector, tracking, and telemetry data."""

from collections import OrderedDict
from typing import Any, Dict, List, Optional, Tuple


COMPONENTS = ("raw", "tracking", "telemetry")


class FrameSampleMatcher:
    """Join one camera frame with all perception outputs by exact source stamp.

    A frame remains pending until every component arrives or its monotonic
    matching deadline expires. Finalized samples are immutable; later payloads
    are counted and ignored so incomplete rows cannot be partly rewritten.
    """

    def __init__(self, deadline_sec: float = 0.5, orphan_ttl_sec: float = 5.0):
        self.deadline_sec = float(deadline_sec)
        self.orphan_ttl_sec = float(orphan_ttl_sec)
        self.pending_frames: Dict[Tuple[int, int], Dict[str, Any]] = {}
        self.pending_components = {name: OrderedDict() for name in COMPONENTS}
        # At defaults, 60s is 120x the matching deadline and 12x orphan TTL,
        # covering normal late/replayed events. At 30Hz this is about 1,800
        # stamps; the explicit cap keeps memory bounded at higher rates too.
        self.finalized_stamp_ttl_sec = max(
            60.0, self.orphan_ttl_sec, 2.0 * self.deadline_sec
        )
        self.max_finalized_stamps = 10_000
        self.finalized_stamps: OrderedDict[Tuple[int, int], float] = OrderedDict()
        self.late_component_count = 0
        self.orphan_component_count = 0
        self.timeout_count = 0
        self.incomplete_flush_count = 0

    def add_frame(self, frame: Dict[str, Any], now: float) -> List[Dict[str, Any]]:
        stamp = tuple(frame["stamp_key"])
        finalized = self.expire(now)
        if stamp in self.finalized_stamps or stamp in self.pending_frames:
            return finalized
        self.pending_frames[stamp] = {"frame": frame, "deadline": now + self.deadline_sec}
        return finalized + self._complete_if_ready(stamp, now)

    def add_component(self, name: str, stamp: Tuple[int, int], payload: Any, now: float) -> List[Dict[str, Any]]:
        if name not in COMPONENTS:
            raise ValueError(f"Unknown frame component: {name}")
        stamp = tuple(stamp)
        finalized = self.expire(now)
        if stamp in self.finalized_stamps:
            self.late_component_count += 1
            return finalized
        records = self.pending_components[name]
        records[stamp] = {"payload": payload, "arrival_monotonic": now}
        records.move_to_end(stamp)
        self._prune_orphans(now)
        return finalized + self._complete_if_ready(stamp, now)

    def expire(self, now: float, force: bool = False) -> List[Dict[str, Any]]:
        finalized = []
        for stamp, record in list(self.pending_frames.items()):
            if force or now >= record["deadline"]:
                frame = record["frame"]
                components = self._take_components(stamp)
                self.pending_frames.pop(stamp, None)
                self._remember_finalized(stamp, now)
                match_status = "incomplete_timeout" if now >= record["deadline"] else "incomplete_flush"
                if match_status == "incomplete_timeout":
                    self.timeout_count += 1
                else:
                    self.incomplete_flush_count += 1
                finalized.append({
                    "frame": frame,
                    "components": components,
                    "match_status": match_status,
                })
        self._prune_finalized(now)
        self._prune_orphans(now)
        return finalized

    def _complete_if_ready(self, stamp: Tuple[int, int], now: float) -> List[Dict[str, Any]]:
        frame_record = self.pending_frames.get(stamp)
        if frame_record is None or any(stamp not in self.pending_components[name] for name in COMPONENTS):
            return []
        components = self._take_components(stamp)
        self.pending_frames.pop(stamp, None)
        self._remember_finalized(stamp, now)
        return [{"frame": frame_record["frame"], "components": components, "match_status": "complete"}]

    def _remember_finalized(self, stamp: Tuple[int, int], now: float) -> None:
        """Tombstone complete, timed-out, and flushed stamps alike."""
        self.finalized_stamps[stamp] = now
        self.finalized_stamps.move_to_end(stamp)
        self._prune_finalized(now)

    def _take_components(self, stamp: Tuple[int, int]) -> Dict[str, Optional[Dict[str, Any]]]:
        return {name: self.pending_components[name].pop(stamp, None) for name in COMPONENTS}

    def _prune_orphans(self, now: float) -> None:
        for name in COMPONENTS:
            records = self.pending_components[name]
            for stamp, record in list(records.items()):
                if stamp not in self.pending_frames and now - record["arrival_monotonic"] > self.orphan_ttl_sec:
                    records.pop(stamp, None)
                    self.orphan_component_count += 1
            while len(records) > 1000:
                records.popitem(last=False)
                self.orphan_component_count += 1

    def _prune_finalized(self, now: float) -> None:
        while self.finalized_stamps:
            stamp, finalized_at = next(iter(self.finalized_stamps.items()))
            if now - finalized_at <= self.finalized_stamp_ttl_sec:
                break
            self.finalized_stamps.pop(stamp, None)
        while len(self.finalized_stamps) > self.max_finalized_stamps:
            self.finalized_stamps.popitem(last=False)
