"""Invalid ORB estimates must not collapse the native tracker state."""
import numpy as np
import pytest

from test_botsort_adapter import _adapter, _boxes, _image, _measurement


@pytest.mark.parametrize('warp', [
    np.zeros((2, 3)),
    np.array([[-1., 0., 0.], [0., 1., 0.]]),
    np.array([[1., 2., 0.], [2., 4., 0.]]),
    np.full((2, 3), np.nan),
    np.eye(3),
])
def test_invalid_gmc_uses_native_fallback_and_preserves_fresh_identity(warp):
    adapter = _adapter()
    box = [20., 20., 70., 120.]
    first = adapter.update(_boxes([box + [.9, 0.]]), _image(), [_measurement(box)])
    adapter._gmc_apply = lambda _image, _detections=None: warp
    moved = [22., 20., 72., 120.]
    observed = adapter.update(_boxes([moved + [.9, 0.]]), _image(1), [_measurement(moved)])
    assert adapter.gmc_fallbacks == 1
    np.testing.assert_array_equal(adapter.last_gmc_warp, np.eye(2, 3))
    assert not adapter.last_gmc_non_identity
    assert observed[0]['track_id'] == first[0]['track_id']
    assert observed[0]['bbox'] == moved
    assert np.isfinite(adapter.tracker.tracked_stracks[0].mean).all()


def test_singular_empty_frame_does_not_poison_lost_track_recovery():
    adapter = _adapter()
    box = [20., 20., 70., 120.]
    first = adapter.update(_boxes([box + [.9, 0.]]), _image(), [_measurement(box)])
    adapter._gmc_apply = lambda _image, _detections=None: np.zeros((2, 3))
    assert adapter.update(_boxes([]), _image(1), []) == []
    assert adapter.gmc_fallbacks == 1
    assert np.isfinite(adapter.tracker.lost_stracks[0].mean).all()
    adapter._gmc_apply = lambda _image, _detections=None: np.eye(2, 3)
    returned = adapter.update(_boxes([box + [.9, 0.]]), _image(2), [_measurement(box)])
    assert returned[0]['track_id'] == first[0]['track_id']
    assert returned[0]['bbox'] == box


def test_valid_camera_translation_is_still_applied_by_native_tracker():
    adapter = _adapter()
    box = [20., 20., 70., 120.]
    first = adapter.update(_boxes([box + [.9, 0.]]), _image(), [_measurement(box)])
    warp = np.array([[1., 0., 25.], [0., 1., 10.]], dtype=np.float32)
    adapter._gmc_apply = lambda _image, _detections=None: warp
    moved = [45., 30., 95., 130.]
    observed = adapter.update(_boxes([moved + [.9, 0.]]), _image(1), [_measurement(moved)])
    assert adapter.gmc_fallbacks == 0
    np.testing.assert_array_equal(adapter.last_gmc_warp, warp)
    assert adapter.last_gmc_non_identity
    assert observed[0]['track_id'] == first[0]['track_id']
    assert observed[0]['bbox'] == moved
