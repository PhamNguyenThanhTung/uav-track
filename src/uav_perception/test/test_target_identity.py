#!/usr/bin/env python3
"""
Unit tests for Target Identity Abstraction (Phase 4A).

Tests TargetState and TargetIdentityManager for persistent target association,
tracker ID switch detection, coasting timeout, and track loss transitions.
"""

import numpy as np
import pytest
from uav_perception.target_identity import TargetIdentityManager, TargetState, TrackingStatus


class MockTrack:
    """Mock tracker observation for unit testing."""
    def __init__(
        self,
        track_id: str,
        ltrb: list,
        conf: float = 0.85,
        time_since_update: int = 0,
        feature=None,
        kalman_ltrb=None,
        velocity=None,
        auxiliary=None,
    ):
        self.track_id = str(track_id)
        self._ltrb = [float(v) for v in ltrb]
        self._kalman_ltrb = [float(v) for v in (kalman_ltrb or ltrb)]
        self._conf = float(conf)
        self.time_since_update = time_since_update
        self._feature = feature if feature is not None else [1.0, 0.0, 0.0]
        self.feature_calls = 0
        self._auxiliary = auxiliary
        if velocity is not None:
            x1, y1, x2, y2 = self._ltrb
            self.mean = [(x1+x2)/2, (y1+y2)/2, (x2-x1)/(y2-y1), y2-y1, *velocity]

    def to_ltrb(self, orig=False, orig_strict=False):
        return self._ltrb if orig else self._kalman_ltrb

    def get_feature(self):
        self.feature_calls += 1
        return self._feature

    def get_det_conf(self):
        return self._conf

    def get_det_supplementary(self):
        return {'appearance_aux': self._auxiliary} if self._auxiliary is not None else None


def test_target_state_dataclass():
    """Verify TargetState default attributes, center calculation, and active state."""
    state = TargetState(target_id="1")
    assert state.target_id == "1"
    assert state.tracker_id is None
    assert state.status == TrackingStatus.SEARCHING
    assert state.center is None
    assert not state.is_active

    state.bbox = [100.0, 200.0, 300.0, 400.0]
    state.status = TrackingStatus.TRACKING
    assert state.center == (200.0, 300.0)
    assert state.is_active


def test_initial_search_to_track():
    """Verify transition from SEARCHING to TRACKING upon first track acquisition."""
    manager = TargetIdentityManager(coasting_timeout_sec=1.5, default_target_id="1")
    assert manager.tracking_status == TrackingStatus.SEARCHING
    assert manager.active_tracker_id is None
    assert manager.tracked_bbox is None

    t0 = 100.0
    tracks = [MockTrack(track_id="42", ltrb=[100, 150, 200, 350], conf=0.91)]
    state = manager.update(tracks, current_time=t0)

    assert state.status == TrackingStatus.TRACKING
    assert state.target_id == "1"
    assert state.tracker_id == "42"
    assert state.bbox == [100.0, 150.0, 200.0, 350.0]
    assert state.confidence == pytest.approx(0.91)
    assert state.last_seen_time == t0
    assert manager.id_switch_count == 0
    assert manager.lost_track_count == 0
    assert manager.pop_id_switch_event() is None


def test_persistent_tracking_same_id():
    """Verify continuous tracking with the same tracker ID updates bbox and timestamp."""
    manager = TargetIdentityManager(coasting_timeout_sec=1.5)
    manager.update([MockTrack("7", [10, 20, 30, 40], 0.8)], current_time=10.0)

    # Frame 2: same ID, updated coordinates
    manager.update([MockTrack("7", [15, 25, 35, 45], 0.85)], current_time=10.1)

    assert manager.tracking_status == TrackingStatus.TRACKING
    assert manager.active_tracker_id == "7"
    assert manager.tracked_bbox == [15.0, 25.0, 35.0, 45.0]
    assert manager.last_track_seen_time == 10.1
    assert manager.id_switch_count == 0


def test_gmc_camera_warp_compensates_target_motion_gate_for_camera_pan():
    start = MockTrack("7", [100, 50, 160, 190], feature=[1, 0, 0])
    moved = MockTrack("7", [155, 50, 215, 190], feature=[1, 0, 0])
    warp = np.asarray([[1, 0, 55], [0, 1, 0]], dtype=np.float32)

    uncompensated = TargetIdentityManager(coasting_timeout_sec=1.5)
    uncompensated.update([start], current_time=10.0)
    plain_state = uncompensated.update([moved], current_time=10.1)

    compensated = TargetIdentityManager(coasting_timeout_sec=1.5)
    compensated.update([start], current_time=10.0)
    compensated_state = compensated.update([moved], current_time=10.1, camera_warp=warp)

    assert plain_state.status == TrackingStatus.UNCERTAIN
    assert compensated_state.status == TrackingStatus.TRACKING
    assert compensated_state.bbox == [155.0, 50.0, 215.0, 190.0]
    assert compensated.accepted_observation_count == 2


def test_tracker_id_switch_detection():
    """Verify detection of tracker ID switch while preserving persistent target identity."""
    manager = TargetIdentityManager(coasting_timeout_sec=1.5, default_target_id="primary_target")
    manager.update([MockTrack("101", [50, 50, 100, 100], 0.9)], current_time=1.0)
    assert manager.active_tracker_id == "101"
    assert manager.id_switch_count == 0

    # Underlying tracker switches ID from 101 to 202
    first_switched = manager.update([MockTrack("202", [55, 55, 105, 105], 0.88)], current_time=1.1)
    assert first_switched.status == TrackingStatus.UNCERTAIN
    assert manager.active_tracker_id == "101"
    manager.update([MockTrack("202", [58, 58, 108, 108], 0.88)], current_time=1.2)

    assert manager.active_target_id == "primary_target"  # Persistent ID preserved
    assert manager.active_tracker_id == "202"           # Tracker ID updated
    assert manager.id_switch_count == 1                 # Switch detected
    assert manager.tracking_status == TrackingStatus.TRACKING

    event = manager.pop_id_switch_event()
    assert event == ("101", "202")
    assert manager.pop_id_switch_event() is None  # Event queue cleared


def test_crossing_freezes_history_and_allows_correct_new_tid_while_old_tid_goes_opposite():
    manager = TargetIdentityManager(coasting_timeout_sec=1.5)
    manager.update([MockTrack("old", [100, 100, 200, 300], feature=[1.0, 0.0])], 10.0)
    manager.update([MockTrack("old", [110, 100, 210, 300], feature=[1.0, 0.0])], 10.1)
    manager.update([MockTrack("old", [120, 100, 220, 300], feature=[1.0, 0.0])], 10.2)
    history = list(manager._history)
    refs = manager.appearance_reference_count
    state = manager.update([
        MockTrack("old", [130, 100, 230, 300], feature=[1.0, 0.0]),
        MockTrack("other", [140, 100, 240, 300], feature=[0.99, 0.141067]),
    ], 10.3)
    assert state.status == TrackingStatus.UNCERTAIN
    assert state.bbox is None
    assert list(manager._history) == history
    assert manager.appearance_reference_count == refs

    # One merged box does not establish that the people have separated.
    manager.update([MockTrack("old", [140, 100, 280, 300], feature=[1.0, 0.0])], 10.4)
    assert manager.tracked_bbox is None
    assert list(manager._history) == history

    for now, shift in [(10.7, 0), (10.8, 10)]:
        state = manager.update([
            MockTrack("old", [40-shift, 100, 140-shift, 300], feature=[0.99, 0.141067], velocity=[-5, 0]),
            MockTrack("new", [170+shift, 100, 270+shift, 300], feature=[0.95, 0.31225], velocity=[5, 0]),
        ], now)
    assert state.status == TrackingStatus.TRACKING
    assert state.tracker_id == "new"
    assert state.target_id == "1"
    assert state.bbox == [180.0, 100.0, 280.0, 300.0]
    assert manager.raw_tid_switch_count == 1
    assert manager.appearance_reference_count == refs


def test_coasting_and_timeout_loss():
    """Verify transition from TRACKING -> COASTING (< 1.5s) -> LOST (>= 1.5s)."""
    manager = TargetIdentityManager(coasting_timeout_sec=1.5)
    t_seen = 20.0
    manager.update([MockTrack("1", [0, 0, 50, 50])], current_time=t_seen)
    assert manager.tracking_status == TrackingStatus.TRACKING

    # 0.5s later: target occluded/missing -> COASTING
    manager.update([], current_time=t_seen + 0.5)
    assert manager.tracking_status == TrackingStatus.COASTING
    assert manager.tracked_bbox is None
    assert manager.lost_track_count == 0

    # 1.4s later: still within coasting window (1.5s) -> COASTING
    manager.update([], current_time=t_seen + 1.4)
    assert manager.tracking_status == TrackingStatus.COASTING
    assert manager.lost_track_count == 0

    # 1.6s later: exceeds 1.5s timeout -> LOST (increments lost count once)
    manager.update([], current_time=t_seen + 1.6)
    assert manager.tracking_status == TrackingStatus.LOST
    assert manager.lost_track_count == 1

    # 2.0s later: remains LOST without repeatedly incrementing lost count
    manager.update([], current_time=t_seen + 2.0)
    assert manager.tracking_status == TrackingStatus.LOST
    assert manager.lost_track_count == 1


def test_single_matching_frame_does_not_reacquire_and_reset_permits_new_selection():
    """Recovery needs confirmation; explicit reset permits immediate new selection."""
    manager = TargetIdentityManager(coasting_timeout_sec=1.5)
    manager.update([MockTrack("1", [0, 0, 50, 50])], current_time=10.0)
    manager.update([], current_time=12.0)
    assert manager.tracking_status == TrackingStatus.LOST
    assert manager.lost_track_count == 1

    manager.update([MockTrack("new", [10, 10, 60, 60])], current_time=13.0)
    assert manager.tracking_status == TrackingStatus.LOST

    manager.reset()
    manager.update([MockTrack("new", [10, 10, 60, 60])], current_time=13.1)
    assert manager.tracking_status == TrackingStatus.TRACKING
    assert manager.tracked_bbox == [10.0, 10.0, 60.0, 60.0]
    assert manager.last_track_seen_time == 13.1


def test_stale_prediction_does_not_refresh_last_seen_or_bbox():
    manager = TargetIdentityManager(coasting_timeout_sec=1.5)
    manager.update([MockTrack("target", [20, 20, 80, 100])], current_time=10.0)

    state = manager.update([
        MockTrack("target", [25, 20, 85, 100], time_since_update=3)
    ], current_time=10.4)

    assert state.status == TrackingStatus.COASTING
    assert state.bbox is None
    assert state.last_seen_time == 10.0


def test_ambiguous_nearby_candidates_abstain_without_reassignment():
    manager = TargetIdentityManager(coasting_timeout_sec=1.5)
    manager.update([MockTrack("old", [100, 100, 200, 250])], current_time=10.0)

    state = manager.update([
        MockTrack("near-a", [100, 100, 190, 245]),
        MockTrack("near-b", [110, 105, 205, 255]),
    ], current_time=10.1)

    assert state.status == TrackingStatus.UNCERTAIN
    assert state.tracker_id == "old"
    assert state.bbox is None
    assert state.id_switch_count == 0
    assert state.lost_count == 0


def test_ambiguity_persists_until_timeout_without_emitting_bbox():
    manager = TargetIdentityManager(coasting_timeout_sec=1.5)
    manager.update([MockTrack("old", [100, 100, 200, 250])], current_time=10.0)

    state = manager.update([
        MockTrack("a", [100, 100, 190, 245], feature=[0.0, 1.0, 0.0]),
        MockTrack("b", [110, 105, 205, 255], feature=[0.0, 1.0, 0.0]),
    ], current_time=10.1)
    assert state.status == TrackingStatus.UNCERTAIN

    for timestamp in (10.2, 10.3, 10.8, 11.5):
        state = manager.update(
            [MockTrack("a", [100, 100, 190, 245], feature=[0.0, 1.0, 0.0])], current_time=timestamp
        )
        assert state.bbox is None
        assert state.tracker_id == "old"
    assert state.status == TrackingStatus.LOST


def test_ambiguous_or_impossible_candidates_stay_lost_without_confirmed_identity():
    manager = TargetIdentityManager(coasting_timeout_sec=1.5)
    manager.update([MockTrack("old", [100, 100, 200, 250])], current_time=10.0)
    manager.update([
        MockTrack("a", [100, 100, 190, 245], feature=[0.0, 1.0, 0.0]),
        MockTrack("b", [110, 105, 205, 255], feature=[0.0, 1.0, 0.0]),
    ], current_time=10.1)

    subsequent_frames = [
        [MockTrack("near", [105, 105, 195, 245], feature=[0.0, 1.0, 0.0])],
        [MockTrack("far", [500, 400, 590, 550], feature=[0.0, 1.0, 0.0])],
        [],
        [MockTrack("another-near", [95, 95, 190, 245], feature=[0.0, 1.0, 0.0])],
    ]
    times = (10.2, 10.6, 11.0, 11.7)
    for tracks, timestamp in zip(subsequent_frames, times):
        state = manager.update(tracks, current_time=timestamp)
        assert state.tracker_id == "old"
        assert state.bbox is None
    assert state.status == TrackingStatus.LOST
    state = manager.update([MockTrack("old", [100, 100, 200, 250])], current_time=11.8)
    assert state.status == TrackingStatus.LOST


def test_explicit_reset_clears_ambiguity_lockout_for_new_selection():
    manager = TargetIdentityManager(coasting_timeout_sec=1.5)
    manager.update([MockTrack("old", [100, 100, 200, 250])], current_time=10.0)
    manager.update([
        MockTrack("a", [100, 100, 190, 245]),
        MockTrack("b", [110, 105, 205, 255]),
    ], current_time=10.1)

    manager.reset()
    state = manager.update(
        [MockTrack("new", [300, 200, 390, 350])], current_time=10.2
    )

    assert state.status == TrackingStatus.TRACKING
    assert state.tracker_id == "new"
    assert state.bbox == [300.0, 200.0, 390.0, 350.0]


def test_normal_temporary_loss_can_reacquire_same_identity():
    manager = TargetIdentityManager(coasting_timeout_sec=1.5)
    manager.update([MockTrack("A", [100, 100, 200, 250])], current_time=10.0)

    state = manager.update([], current_time=10.1)
    assert state.status == TrackingStatus.COASTING

    state = manager.update(
        [MockTrack("A", [102, 100, 202, 250])], current_time=10.2
    )
    assert state.status == TrackingStatus.TRACKING
    assert state.tracker_id == "A"


def test_stale_prediction_is_not_an_observation_during_ambiguity():
    manager = TargetIdentityManager(coasting_timeout_sec=1.5)
    manager.update([MockTrack("old", [100, 100, 200, 250])], current_time=10.0)
    manager.update([
        MockTrack("a", [100, 100, 190, 245]),
        MockTrack("b", [110, 105, 205, 255]),
    ], current_time=10.1)

    state = manager.update(
        [MockTrack("a", [100, 100, 190, 245], time_since_update=1)],
        current_time=10.2,
    )

    assert state.status == TrackingStatus.PREDICTING
    assert state.tracker_id == "old"
    assert state.bbox is None
    assert state.last_seen_time == 10.0
    assert state.lost_count == 0


def test_observation_at_timestamp_zero_can_coast():
    manager = TargetIdentityManager(coasting_timeout_sec=1.5)
    manager.update([MockTrack("zero", [0, 0, 50, 50])], current_time=0.0)

    state = manager.update([], current_time=0.5)

    assert state.status == TrackingStatus.COASTING
    assert state.last_seen_time == 0.0


def test_manager_reset():
    """Verify manager reset restores pristine initial state."""
    manager = TargetIdentityManager()
    manager.update([MockTrack("1", [0, 0, 50, 50])], current_time=5.0)
    manager.reset()

    assert manager.active_tracker_id is None
    assert manager.tracked_bbox is None
    assert manager.tracking_status == TrackingStatus.SEARCHING
    assert manager.id_switch_count == 0
    assert manager.lost_track_count == 0
    assert manager.last_track_seen_time == 0.0


def test_wrong_person_is_rejected_even_when_tracker_reuses_the_same_tid():
    manager = TargetIdentityManager()
    manager.update(
        [MockTrack("7", [100, 100, 200, 250], feature=[1.0, 0.0, 0.0])],
        current_time=10.0,
    )

    state = manager.update(
        [MockTrack("7", [102, 100, 202, 250], feature=[0.0, 1.0, 0.0])],
        current_time=10.1,
    )

    assert state.status == TrackingStatus.UNCERTAIN
    assert state.tracker_id == "7"
    assert state.bbox is None
    assert manager.raw_tid_switch_count == 0


def test_target_identity_can_move_to_new_tid_after_two_valid_observations():
    manager = TargetIdentityManager()
    manager.update(
        [MockTrack("old", [100, 100, 200, 250], feature=[1.0, 0.0, 0.0])],
        current_time=10.0,
    )

    first = manager.update(
        [MockTrack("new", [108, 100, 208, 250], feature=[0.99, 0.10, 0.0])],
        current_time=10.1,
    )
    assert first.status == TrackingStatus.UNCERTAIN
    assert first.bbox is None
    assert first.tracker_id == "old"

    state = manager.update(
        [MockTrack("new", [112, 100, 212, 250], feature=[0.99, 0.10, 0.0])],
        current_time=10.2,
    )
    assert state.status == TrackingStatus.TRACKING
    assert state.target_id == "1"
    assert state.tracker_id == "new"
    assert manager.raw_tid_switch_count == 1
    assert manager.appearance_reference_count == 1
    assert manager.pop_id_switch_event() == ("old", "new")


def test_strong_overlap_never_confirms_or_updates_gallery_even_with_unique_embedding():
    manager = TargetIdentityManager()
    manager.update(
        [MockTrack("old", [100, 100, 200, 250], feature=[1.0, 0.0, 0.0])],
        current_time=10.0,
    )
    assert manager.appearance_reference_count == 1

    def crossing_tracks(dx):
        return [
            MockTrack("new", [100 + dx, 100, 200 + dx, 250], feature=[0.99, 0.10, 0.0]),
            MockTrack("other", [120 + dx, 100, 220 + dx, 250], feature=[0.0, 1.0, 0.0]),
        ]

    first = manager.update(crossing_tracks(0), current_time=10.1)
    assert first.status == TrackingStatus.UNCERTAIN
    assert manager.appearance_reference_count == 1
    assert first.bbox is None

    state = manager.update(crossing_tracks(2), current_time=10.2)
    assert state.status == TrackingStatus.UNCERTAIN
    assert state.tracker_id == "old"
    assert state.bbox is None
    assert manager.appearance_reference_count == 1


def test_dirty_candidates_with_tied_appearance_and_motion_abstain():
    manager = TargetIdentityManager()
    manager.update(
        [MockTrack("old", [100, 100, 200, 250], feature=[1.0, 0.0, 0.0])],
        current_time=10.0,
    )
    similar = [0.99, 0.10, 0.0]

    state = manager.update(
        [
            MockTrack("candidate-a", [100, 100, 200, 250], feature=similar),
            MockTrack("candidate-b", [110, 100, 210, 250], feature=similar),
        ],
        current_time=10.1,
    )

    assert state.status == TrackingStatus.UNCERTAIN
    assert state.tracker_id == "old"
    assert state.bbox is None
    assert manager.appearance_reference_count == 1


def test_candidate_beyond_short_motion_gate_is_rejected_even_with_matching_feature():
    manager = TargetIdentityManager(max_reacquisition_distance_px=100.0)
    manager.update(
        [MockTrack("target", [100, 100, 200, 250], feature=[1.0, 0.0, 0.0])],
        current_time=10.0,
    )

    state = manager.update(
        [MockTrack("target", [205, 100, 305, 250], feature=[1.0, 0.0, 0.0])],
        current_time=10.1,
    )

    assert state.status == TrackingStatus.UNCERTAIN
    assert state.tracker_id == "target"
    assert state.bbox is None


def test_motion_jump_is_abstained_even_with_a_matching_feature():
    manager = TargetIdentityManager()
    manager.update(
        [MockTrack("target", [100, 100, 200, 250], feature=[1.0, 0.0, 0.0])],
        current_time=10.0,
    )

    state = manager.update(
        [MockTrack("target", [1000, 100, 1100, 250], feature=[1.0, 0.0, 0.0])],
        current_time=10.1,
    )

    assert state.status == TrackingStatus.UNCERTAIN
    assert state.tracker_id == "target"
    assert state.bbox is None


def test_implausible_step_does_not_poison_motion_history():
    manager = TargetIdentityManager()
    feature = [1.0, 0.0, 0.0]
    manager.update(
        [MockTrack("target", [100, 100, 200, 250], feature=feature)],
        current_time=10.0,
    )
    first = manager.update(
        [MockTrack("target", [175, 100, 275, 250], feature=feature)],
        current_time=10.1,
    )
    assert first.status == TrackingStatus.UNCERTAIN

    # The unverified 750 px/s jump must not become the target's trajectory.
    for now, dx in [(10.2, 5), (10.3, 8)]:
        state = manager.update([MockTrack("target", [100+dx, 100, 200+dx, 250], feature=feature)], now)
    assert state.status == TrackingStatus.TRACKING
    assert state.bbox == [108.0, 100.0, 208.0, 250.0]


def test_similar_appearance_on_another_vertical_path_is_not_the_target():
    manager = TargetIdentityManager()
    manager.update([MockTrack("old", [100, 100, 200, 300])], 10.0)
    manager.update([MockTrack("old", [110, 100, 210, 300])], 10.1)
    state = manager.update([MockTrack("old", [120, 20, 220, 220])], 10.2)
    assert state.status == TrackingStatus.UNCERTAIN
    assert state.bbox is None


def test_missing_observations_decay_confidence_and_late_reappearance_needs_confirmation():
    manager = TargetIdentityManager(coasting_timeout_sec=1.5)
    manager.update([MockTrack("target", [100, 100, 200, 300], conf=0.9)], 10.0)
    state = manager.update([], 10.5)
    assert state.confidence == pytest.approx(0.6)
    state = manager.update([MockTrack("target", [100, 100, 200, 300], conf=0.9)], 11.6)
    assert state.status == TrackingStatus.LOST
    assert state.bbox is None
    assert state.confidence == 0.0


def test_manager_reuses_embedding_and_uses_original_detection_bbox():
    manager = TargetIdentityManager()
    track = MockTrack(
        "target",
        [100, 100, 200, 250],
        feature=[1.0, 0.0, 0.0],
        kalman_ltrb=[700, 400, 800, 550],
    )

    state = manager.update([track], current_time=10.0)

    assert state.bbox == [100.0, 100.0, 200.0, 250.0]
    assert track.feature_calls == 1
    assert manager.appearance_reference_count == 1


def test_duplicate_source_stamp_abstains_without_refreshing_history():
    manager = TargetIdentityManager()
    manager.update(
        [MockTrack("target", [100, 100, 200, 250], feature=[1.0, 0.0, 0.0])],
        current_time=10.0,
    )

    state = manager.update(
        [MockTrack("target", [102, 100, 202, 250], feature=[1.0, 0.0, 0.0])],
        current_time=10.0,
    )

    assert state.status == TrackingStatus.UNCERTAIN
    assert state.bbox is None
    assert state.last_seen_time == 10.0


def test_walking_pose_width_change_is_not_a_scale_jump():
    manager = TargetIdentityManager()
    manager.update([MockTrack("target", [100, 100, 300, 400])], 10.0)
    state = manager.update([MockTrack("target", [170, 100, 230, 400])], 10.1)
    assert state.status == TrackingStatus.TRACKING
    assert state.bbox == [170.0, 100.0, 230.0, 400.0]
    assert state.last_seen_time == 10.1


def test_duplicate_unconfirmed_frame_cannot_complete_tid_reassignment():
    manager = TargetIdentityManager()
    manager.update([MockTrack("old", [100, 100, 200, 300])], 10.0)
    manager.update([MockTrack("new", [102, 100, 202, 300])], 10.1)
    state = manager.update([MockTrack("new", [102, 100, 202, 300])], 10.1)
    assert state.status == TrackingStatus.UNCERTAIN
    assert state.tracker_id == "old"
    assert manager.raw_tid_switch_count == 0
    manager.update([MockTrack("new", [104, 100, 204, 300])], 10.2)
    assert manager.active_tracker_id == "old"
    manager.update([MockTrack("new", [106, 100, 206, 300])], 10.3)
    assert manager.active_tracker_id == "new"


def test_source_clock_reset_ends_identity_session():
    manager = TargetIdentityManager()
    manager.update([MockTrack("old", [100, 100, 200, 300])], 10.0)
    state = manager.update([MockTrack("old", [100, 100, 200, 300])], 1.0)
    assert state.status == TrackingStatus.LOST
    assert state.bbox is None


def test_visible_local_turn_can_resume_after_two_clean_same_person_observations():
    manager = TargetIdentityManager()
    for now, x in [(10.0, 100), (10.1, 90), (10.2, 85)]:
        manager.update([MockTrack("target", [x, 100, x+100, 300], velocity=[-3, 0])], now)
    manager.update([
        MockTrack("target", [90, 100, 190, 300]),
        MockTrack("other", [100, 100, 200, 300]),
    ], 10.3)
    assert manager.tracked_bbox is None
    for now, x in [(10.4, 96), (10.5, 98)]:
        state = manager.update([
            MockTrack("target", [x, 100, x+100, 300], velocity=[3, 0]),
            MockTrack("other", [350, 100, 450, 300], velocity=[3, 0]),
        ], now)
    assert state.status == TrackingStatus.TRACKING
    assert state.tracker_id == "target"
    assert state.bbox == [98.0, 100.0, 198.0, 300.0]
    assert manager.raw_tid_switch_count == 0


def test_overlap_with_unconfirmed_detector_person_blocks_output_and_reference_updates():
    manager = TargetIdentityManager()
    manager.update([MockTrack("target", [100, 100, 200, 300])], 10.0)
    refs = manager.appearance_reference_count
    history = list(manager._history)
    state = manager.update(
        [MockTrack("target", [102, 100, 202, 300])], 10.1,
        detection_bboxes=[[102, 100, 202, 300], [110, 100, 210, 300]],
    )
    assert state.status == TrackingStatus.UNCERTAIN
    assert state.bbox is None
    assert manager.appearance_reference_count == refs
    assert list(manager._history) == history


def test_auxiliary_anchor_rejects_wrong_person_with_same_tid_and_similar_neural_feature():
    manager = TargetIdentityManager()
    manager.update([MockTrack("old", [100, 100, 200, 300], auxiliary=[1, 0])], 10.0)
    state = manager.update([MockTrack("old", [102, 100, 202, 300], auxiliary=[0, 1])], 10.1)
    assert state.status == TrackingStatus.UNCERTAIN
    assert state.bbox is None
    for now in [10.2, 10.3]:
        state = manager.update([
            MockTrack("old", [102, 100, 202, 300], auxiliary=[0, 1]),
            MockTrack("new", [230, 100, 330, 300], auxiliary=[1, 0]),
        ], now)
    # Correct appearance cannot override an implausible position jump.
    assert state.bbox is None
    assert state.tracker_id == "old"


def test_auxiliary_anchor_never_forces_a_choice_between_indistinguishable_candidates():
    manager = TargetIdentityManager()
    manager.update([MockTrack("old", [100, 100, 200, 300], auxiliary=[1, 0])], 10.0)
    state = manager.update([
        MockTrack("a", [80, 100, 100, 300], auxiliary=[1, 0]),
        MockTrack("b", [200, 100, 220, 300], auxiliary=[1, 0]),
    ], 11.0)
    assert state.status == TrackingStatus.UNCERTAIN
    assert state.bbox is None


def test_missing_auxiliary_cue_does_not_silently_bypass_an_enrolled_anchor():
    manager = TargetIdentityManager()
    manager.update([MockTrack("old", [100, 100, 200, 300], auxiliary=[1, 0])], 10.0)
    state = manager.update([MockTrack("old", [100, 100, 200, 300])], 10.1)
    assert state.bbox is None


def test_initial_selection_waits_for_a_clean_reference():
    manager = TargetIdentityManager()
    state = manager.update([
        MockTrack("first", [100, 100, 200, 300]),
        MockTrack("other", [110, 100, 210, 300]),
    ], 10.0)
    assert state.status == TrackingStatus.SEARCHING
    assert state.bbox is None
    assert manager.appearance_reference_count == 0


def test_correct_auxiliary_target_can_change_tid_when_old_tid_belongs_to_someone_else():
    manager = TargetIdentityManager()
    manager.update([MockTrack("old", [100, 100, 200, 300], auxiliary=[1, 0])], 10.0)
    for now, dx in [(10.1, 2), (10.2, 4)]:
        state = manager.update([
            MockTrack("old", [350, 100, 450, 300], auxiliary=[0, 1]),
            MockTrack("other", [360, 100, 460, 300], auxiliary=[0, 1]),
            MockTrack("new", [100+dx, 100, 200+dx, 300], auxiliary=[1, 0]),
        ], now)
    assert state.status == TrackingStatus.TRACKING
    assert state.tracker_id == "new"
    assert state.bbox == [104.0, 100.0, 204.0, 300.0]
    assert state.target_id == "1"


def test_reference_budget_retains_the_enrollment_anchor_without_unbounded_growth():
    manager = TargetIdentityManager(max_appearance_references=2)
    manager.update([MockTrack("target", [100, 100, 200, 300], feature=[1, 0], auxiliary=[1, 0])], 10.0)
    for i in range(1, 12):
        manager.update([MockTrack("target", [100, 100, 200, 300], feature=[0.98, 0.199], auxiliary=[1, 0])], 10.0 + i*.1)
    assert manager.appearance_reference_count == 2
    assert manager._appearance_similarity(np.array([1., 0.])) == pytest.approx(1.)


def test_target_exit_preserves_identity_but_does_not_immediately_lock_a_returned_person():
    manager = TargetIdentityManager(coasting_timeout_sec=1.5)
    manager.update([MockTrack("target", [100, 100, 200, 300], auxiliary=[1, 0])], 10.0)
    for now in [10.1, 10.8, 11.6]:
        state = manager.update([MockTrack("other", [100, 100, 200, 300], auxiliary=[0, 1])], now)
        assert state.bbox is None
    assert state.status == TrackingStatus.LOST
    assert manager.appearance_reference_count == 1
    state = manager.update([MockTrack("returned", [100, 100, 200, 300], auxiliary=[1, 0])], 11.8)
    assert state.status == TrackingStatus.LOST
    assert state.bbox is None


def test_disappearing_neighbor_blocks_contaminated_crop_until_association_is_stable():
    manager = TargetIdentityManager()
    target = MockTrack("target", [100, 100, 200, 300], auxiliary=[1, 0])
    manager.update([target], 10.0, detection_bboxes=[[100, 100, 200, 300], [300, 100, 400, 300], [500, 100, 600, 300]])
    # Torso regions are separate, but the full body crop contains the neighbor.
    boxes = [[100, 100, 200, 300], [170, 100, 270, 300], [500, 100, 600, 300]]
    manager.update([target], 10.1, detection_bboxes=boxes)
    refs = manager.appearance_reference_count
    history = list(manager._history)
    state = manager.update([target], 10.2, detection_bboxes=boxes[:2])
    assert state.status == TrackingStatus.UNCERTAIN
    assert state.bbox is None
    assert manager.appearance_reference_count == refs
    assert list(manager._history) == history
    # Two fresh, stable observations permit the same physical target again.
    for now in (10.3, 10.4):
        state = manager.update([target, MockTrack("neighbor", boxes[1], auxiliary=[0, 1])],
                               now, detection_bboxes=boxes[:2])
    assert state.status == TrackingStatus.TRACKING
    assert state.tracker_id == "target"


def test_clean_target_is_not_blocked_when_a_distant_person_leaves():
    manager = TargetIdentityManager()
    target = MockTrack("target", [100, 100, 200, 300])
    manager.update([target], 10.0, detection_bboxes=[[100, 100, 200, 300], [500, 100, 600, 300]])
    state = manager.update([target], 10.1, detection_bboxes=[[100, 100, 200, 300]])
    assert state.status == TrackingStatus.TRACKING


@pytest.mark.parametrize("clipped_edge", ["bottom", "top"])
def test_partial_target_boxes_do_not_teach_motion_and_full_body_resumes_with_new_tid(clipped_edge):
    manager = TargetIdentityManager()
    for i in range(6):
        manager.update([MockTrack("old", [100+5*i, 100, 200+5*i, 400],
                                  auxiliary=[1, 0])], 10.0 + i*.1)
    trusted = list(manager._history)
    references = manager.appearance_reference_count
    for now, x, height in [(10.6, 130, 270), (10.7, 135, 240)]:
        y1, y2 = (100, 100+height) if clipped_edge == "bottom" else (400-height, 400)
        state = manager.update([
            MockTrack("old", [x, y1, x+100, y2], auxiliary=[1, 0]),
            MockTrack("neighbor", [x+80, 100, x+180, 400], auxiliary=[0, 1]),
        ], now)
        assert state.status == TrackingStatus.TRACKING
    assert list(manager._history) == trusted
    assert manager.appearance_reference_count == references
    manager.update([MockTrack("neighbor", [500, 100, 600, 400], auxiliary=[0, 1])], 10.9)
    for now, x in [(11.4, 170), (11.5, 175)]:
        state = manager.update([
            MockTrack("old", [500, 100, 600, 400], auxiliary=[0, 1]),
            MockTrack("new", [x, 100, x+100, 400], auxiliary=[1, 0], velocity=[5, 0]),
        ], now)
    assert state.status == TrackingStatus.TRACKING
    assert state.tracker_id == "new"
    assert state.target_id == "1"
    assert state.bbox == [175., 100., 275., 400.]


def test_coherent_vertical_translation_under_limb_overlap_keeps_motion_history_current():
    manager = TargetIdentityManager()
    manager.update([MockTrack("target", [100, 100, 200, 400], auxiliary=[1, 0])], 10.0)
    state = manager.update([
        MockTrack("target", [100, 110, 200, 410], auxiliary=[1, 0]),
        MockTrack("neighbor", [180, 110, 280, 410], auxiliary=[0, 1]),
    ], 10.1)
    assert state.status == TrackingStatus.TRACKING
    assert manager._history[-1] == (10.1, [100., 110., 200., 410.])
