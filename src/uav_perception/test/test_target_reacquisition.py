"""Behavioral recovery and identity safety, independent of actor labels or Gazebo."""

import pytest

from uav_perception.target_identity import TargetIdentityManager, TrackingStatus


def observation(tid='old', x=100, feature=(1., 0.), aux=(1., 0.), confidence=.95, age=0, bbox=None):
    return dict(track_id=tid, bbox=bbox or [x, 100, x+100, 300], confidence=confidence,
                feature=feature, appearance_aux=aux, time_since_update=age)


def enrolled():
    manager = TargetIdentityManager(default_target_id='physical-target')
    manager.update([observation()], 10.)
    return manager


def lost():
    manager = enrolled()
    manager.update([], 12.)
    assert manager.tracking_status == TrackingStatus.LOST
    return manager


def confirm(manager, candidate, times=(12.1, 12.2, 12.3)):
    for now in times:
        state = manager.update([candidate], now)
    return state


def test_same_track_continuation_and_short_occlusion_keep_identity():
    manager = enrolled()
    assert manager.update([observation(x=102)], 10.1).is_active
    manager.update([], 10.5)
    state = manager.update([observation(x=105)], 10.6)
    assert state.is_active and state.tracker_id == 'old'


@pytest.mark.parametrize('tid', ['old', 'new'])
def test_recovery_requires_three_fresh_confirmations_and_maps_raw_id_to_same_target(tid):
    manager = lost()
    candidate = observation(tid=tid, x=600)
    for now in (12.1, 12.2):
        state = manager.update([candidate], now)
        assert state.status == TrackingStatus.LOST and state.bbox is None
    state = manager.update([candidate], 12.3)
    assert state.is_active and state.tracker_id == tid
    assert state.target_id == 'physical-target'
    assert state.last_seen_time == 12.3


def test_weak_reid_cannot_reacquire_even_if_color_matches_and_person_is_alone():
    manager = lost()
    state = confirm(manager, observation(tid='wrong', feature=(0., 1.)))
    assert state.status == TrackingStatus.LOST and state.tracker_id == 'old'


def test_crossing_distractor_cannot_take_old_id_or_poison_recovery():
    manager = enrolled()
    manager.update([observation(x=110), observation(tid='other', x=115, aux=(0., 1.))], 10.1)
    assert manager.tracked_bbox is None
    for now in (10.2, 10.8, 12.):
        manager.update([observation(aux=(0., 1.))], now)
        assert manager.tracked_bbox is None
    assert confirm(manager, observation(tid='new', x=150)).is_active
    assert manager.active_tracker_id == 'new'


def test_two_competing_matches_keep_lost_even_when_one_barely_misses_threshold():
    manager = lost()
    for now in (12.1, 12.2, 12.3, 13.):
        state = manager.update([
            observation(tid='a', x=100, feature=(.78, .625)),
            observation(tid='b', x=500, feature=(.75, .661)),
        ], now)
        assert state.status == TrackingStatus.LOST and state.bbox is None


def test_long_absence_does_not_make_old_position_a_permanent_veto():
    manager = lost()
    state = confirm(manager, observation(tid='returned', x=950), times=(100., 100.1, 100.2))
    assert state.is_active and state.target_id == 'physical-target'


@pytest.mark.parametrize('contamination', ['low_confidence', 'border_crop'])
def test_unclean_or_low_confidence_tracking_does_not_teach_new_identity(contamination):
    manager = TargetIdentityManager()
    box = [100, 100, 200, 700]
    manager.update([observation(bbox=box)], 10., image_size=(1280, 720))
    bad = observation(feature=(.8, .6), confidence=.4 if contamination=='low_confidence' else .95,
                      bbox=[100,100,200,720] if contamination=='border_crop' else box)
    manager.update([bad], 10.1, image_size=(1280,720))
    manager.update([], 12.)
    for now in (12.1,12.2,12.3,12.4):
        state = manager.update([observation(tid='contaminant', feature=(.6,.8), bbox=box)],
                               now, image_size=(1280,720))
        assert state.status == TrackingStatus.LOST and state.bbox is None


def test_stale_observations_and_duplicate_time_cannot_complete_recovery():
    manager = lost()
    assert confirm(manager, observation(tid='new', age=1)).status == TrackingStatus.LOST
    manager.update([observation(tid='new')], 12.4)
    manager.update([observation(tid='new')], 12.5)
    manager.update([observation(tid='new')], 12.5)
    assert manager.update([observation(tid='new')],12.6).status == TrackingStatus.LOST
    assert manager.update([observation(tid='new')],12.7).status == TrackingStatus.LOST
    assert manager.update([observation(tid='new')],12.8).is_active


def test_clock_rewind_invalidates_identity_and_requires_explicit_reset():
    manager = enrolled()
    manager.update([], 1.)
    assert confirm(manager, observation(tid='new'), (2.,2.1,2.2)).status == TrackingStatus.LOST
    manager.reset()
    assert manager.update([observation(tid='different', feature=(0.,1.))], 2.3).is_active


def test_missing_auxiliary_evidence_requires_strong_reid():
    manager = TargetIdentityManager()
    manager.update([observation(aux=None)],10.)
    manager.update([],12.)
    assert confirm(manager, observation(tid='lookalike',aux=None,feature=(.8,.6))).status == TrackingStatus.LOST
    assert confirm(manager, observation(tid='new',aux=None),(12.4,12.5,12.6)).is_active


def test_clean_partial_bbox_does_not_create_false_vertical_velocity():
    manager = enrolled()
    # No overlap: changing pose crops only the lower extent, not target motion.
    assert manager.update([observation(bbox=[100,100,200,285])],10.1).is_active
    assert manager.update([observation(bbox=[100,100,200,275])],10.2).is_active
    state = manager.update([observation(bbox=[100,100,200,300])],10.4)
    assert state.is_active and state.tracker_id == 'old'


def test_recovered_target_pose_jitter_does_not_define_two_sample_velocity():
    manager = lost()
    confirm(manager, observation(tid='new'))
    assert manager.update([observation(tid='new',bbox=[100,110,200,310])],12.4).is_active
    state = manager.update([observation(tid='new',bbox=[100,100,200,300])],12.65)
    assert state.is_active and state.tracker_id == 'new'


def test_persistent_vertical_projection_error_recovers_with_coarse_spatial_and_strong_identity():
    manager = enrolled()
    candidate = observation(tid='new',bbox=[100,140,200,340])
    assert manager.update([candidate],10.1).bbox is None
    for now in (10.6,10.7):
        assert manager.update([candidate],now).bbox is None
    state = manager.update([candidate],10.8)
    assert state.is_active and state.target_id == 'physical-target'


def test_short_interruption_does_not_ignore_coarse_position_for_far_lookalike():
    manager = enrolled()
    for now in (10.6,10.7,10.8):
        state = manager.update([observation(tid='far',x=900)],now)
        assert state.bbox is None and state.tracker_id == 'old'


def test_three_clean_matches_can_bridge_uncounted_pose_noise_without_following_bad_frame():
    manager = lost()
    strong = observation(tid='new')
    weak = observation(tid='new',feature=(.65,.76))
    assert manager.update([strong],12.1).bbox is None
    assert manager.update([weak],12.2).bbox is None
    assert manager.update([strong],12.3).bbox is None
    assert manager.update([weak],12.4).bbox is None
    assert manager.update([strong],12.5).is_active


def test_counterevidence_resets_positive_sequence_and_bad_frames_never_count():
    manager = lost()
    strong = observation(tid='new')
    manager.update([strong],12.1)
    for now in (12.2,12.3):
        assert manager.update([observation(tid='new',feature=(.65,.76))],now).bbox is None
    manager.update([observation(tid='new',aux=(0.,1.))],12.4)
    assert manager.update([strong],12.5).bbox is None
    assert manager.update([strong],12.6).bbox is None
    assert manager.update([strong],12.7).is_active


def test_confirmation_memory_expires_despite_continuous_unconfirmed_track():
    manager = lost()
    manager.update([observation(tid='new')],12.1)
    for i in range(1,27):
        assert manager.update([observation(tid='new',feature=(.65,.76))],12.1+i*.1).bbox is None
    assert manager.update([observation(tid='new')],14.9).bbox is None
