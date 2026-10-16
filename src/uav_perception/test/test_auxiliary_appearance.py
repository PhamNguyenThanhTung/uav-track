import numpy as np

from uav_perception.image_utils import torso_histogram


def test_torso_descriptor_is_generic_and_independent_of_brightness():
    red = np.full((100, 100, 3), (0, 0, 200), dtype=np.uint8)
    dim_red = np.full((100, 100, 3), (0, 0, 100), dtype=np.uint8)
    green = np.full((100, 100, 3), (0, 200, 0), dtype=np.uint8)
    a, b, c = [torso_histogram(im) for im in [red, dim_red, green]]
    assert a.shape == (192,)
    assert np.isclose(np.linalg.norm(a), 1.0)
    assert np.dot(a, b) > 0.99
    assert np.dot(a, c) < 0.01


def test_gray_or_empty_crops_produce_no_auxiliary_identity_evidence():
    assert not torso_histogram(np.full((100, 100, 3), 128, dtype=np.uint8)).any()
    assert not torso_histogram(np.zeros((0, 0, 3), dtype=np.uint8)).any()


def test_background_outside_the_existing_torso_crop_cannot_change_reference():
    image = np.full((100, 100, 3), (0, 200, 0), dtype=np.uint8)
    image[20:60, 20:80] = (0, 0, 200)
    torso_only = np.full((100, 100, 3), (0, 0, 200), dtype=np.uint8)
    assert np.dot(torso_histogram(image), torso_histogram(torso_only)) > 0.99
