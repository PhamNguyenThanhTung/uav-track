"""CUDA equivalence and fallback checks; opt in with the real ReID runtime suite."""
import os
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np
import pytest
import torch

from uav_perception.gpu_inference import CUDAGraphForward


def test_cuda_graph_rejects_cpu_capture():
    with pytest.raises(ValueError, match='CUDA example'):
        CUDAGraphForward(lambda x: x + 1, torch.ones(2))


def _cuda_runtime():
    if os.environ.get('RUN_BOTSORT_REID_RUNTIME') != '1':
        pytest.skip('set RUN_BOTSORT_REID_RUNTIME=1 for CUDA checks')
    assert torch.cuda.is_available(), 'CUDA runtime requested but unavailable'


def test_cuda_graph_changing_inputs_worker_thread_and_permanent_fallback():
    _cuda_runtime()
    example = torch.ones((1, 3, 8, 8), device='cuda')
    def eager(x, **kwargs):
        return x.square() + 2
    replay = CUDAGraphForward(eager, example)
    with ThreadPoolExecutor(max_workers=1) as worker:
        for value in (0., 1., -3., 8.):
            image = torch.full_like(example, value)
            actual = worker.submit(replay, image, augment=False, embed=None).result().clone()
            torch.testing.assert_close(actual, eager(image), rtol=0, atol=0)
    assert replay.enabled
    changed = torch.ones((1, 3, 12, 8), device='cuda')
    torch.testing.assert_close(replay(changed), eager(changed), rtol=0, atol=0)
    assert not replay.enabled
    torch.testing.assert_close(replay(example), eager(example), rtol=0, atol=0)
    assert not replay.enabled


def test_cuda_graph_prediction_mode_change_uses_eager():
    _cuda_runtime()
    image = torch.ones((4,), device='cuda')
    def eager(x, augment=False):
        return x + (5 if augment else 1)
    replay = CUDAGraphForward(eager, image)
    torch.testing.assert_close(replay(image, augment=True), image + 5, rtol=0, atol=0)
    assert not replay.enabled


def test_actual_yolo_cuda_graph_matches_changing_images_and_keeps_results():
    _cuda_runtime()
    from ultralytics import YOLO
    model = YOLO(str(Path.home() / '.cache/ultralytics/yolo26n.pt'))
    rng = np.random.default_rng(20261015)
    images = [rng.integers(0, 256, (720, 1280, 3), dtype=np.uint8) for _ in range(3)]
    options = dict(device='cuda:0', imgsz=640, conf=.00001, classes=[0], max_det=100, verbose=False)
    expected = [model.predict(image, **options)[0].boxes.data.clone() for image in images]
    assert all(len(boxes) > 0 for boxes in expected)
    network = model.predictor.model.backend.model
    example = model.predictor.preprocess([images[0]])
    assert tuple(example.shape) == (1, 3, 384, 640)
    network.forward = CUDAGraphForward(network.forward, example)
    retained = []
    for image, boxes in zip(images, expected):
        result = model.predict(image, **options)[0]
        retained.append(result)
        torch.testing.assert_close(result.boxes.data, boxes, rtol=0, atol=0)
    # Results must own their postprocessed detections after subsequent replays.
    for result, boxes in zip(retained, expected):
        torch.testing.assert_close(result.boxes.data, boxes, rtol=0, atol=0)
    assert network.forward.enabled
