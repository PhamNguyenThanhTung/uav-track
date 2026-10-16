import numpy as np

from uav_perception.image_utils import image_to_bgr, bgr_to_ros_bytes


def test_rgb_and_bgr_keep_known_red_and_blue_values_at_ros_boundary():
    red_rgb = bytes((255, 0, 0))
    red_bgr = image_to_bgr(red_rgb, 'rgb8', 1, 1, 3)
    assert red_bgr[0, 0].tolist() == [0, 0, 255]
    assert bgr_to_ros_bytes(red_bgr, 'rgb8') == red_rgb

    blue_bgr_bytes = bytes((255, 0, 0))
    blue_bgr = image_to_bgr(blue_bgr_bytes, 'bgr8', 1, 1, 3)
    assert blue_bgr[0, 0].tolist() == [255, 0, 0]
    assert bgr_to_ros_bytes(blue_bgr, 'bgr8') == blue_bgr_bytes
    assert bgr_to_ros_bytes(blue_bgr, 'rgb8') == bytes((0, 0, 255))


def test_decoder_skips_ros_row_padding():
    # Two rgb8 pixels per row (6 data bytes) with 2 padding bytes per row.
    row_1 = bytes((255, 0, 0, 0, 0, 255, 99, 98))
    row_2 = bytes((0, 255, 0, 255, 255, 0, 97, 96))
    decoded = image_to_bgr(row_1 + row_2, 'rgb8', width=2, height=2, step=8)

    assert decoded.shape == (2, 2, 3)
    assert decoded[0].tolist() == [[0, 0, 255], [255, 0, 0]]
    assert decoded[1].tolist() == [[0, 255, 0], [0, 255, 255]]


def test_annotated_output_supports_mono_encoding():
    bgr = np.array([[[0, 0, 255]]], dtype=np.uint8)
    assert len(bgr_to_ros_bytes(bgr, 'mono8')) == 1
