"""Replay a warmed, fixed-shape CUDA model without Python kernel dispatch."""

import torch


class CUDAGraphForward:
    """Serialize calls and consume outputs before the next call (shared buffers).

    Capture keeps the existing model, precision and operations. A different
    tensor signature or prediction mode permanently returns to eager execution:
    changing shapes can replace model caches referenced by the captured graph.
    """

    def __init__(self, forward, example):
        if example.device.type != 'cuda':
            raise ValueError('CUDA Graph requires a CUDA example tensor')
        self._forward = forward
        self._input = torch.empty_like(example)
        self._input.copy_(example)
        self.enabled = True
        stream = torch.cuda.Stream(device=example.device)
        stream.wait_stream(torch.cuda.current_stream(example.device))
        with torch.cuda.stream(stream), torch.inference_mode():
            for _ in range(3):
                forward(self._input)
        torch.cuda.current_stream(example.device).wait_stream(stream)
        self._graph = torch.cuda.CUDAGraph()
        with torch.inference_mode(), torch.cuda.graph(self._graph):
            self._output = forward(self._input)

    def __call__(self, image, *args, **kwargs):
        supported = (
            not args
            and all((key == 'augment' and value is False)
                    or (key == 'visualize' and value is False)
                    or (key == 'embed' and value is None)
                    for key, value in kwargs.items())
            and image.shape == self._input.shape
            and image.dtype == self._input.dtype
            and image.device == self._input.device
        )
        if not self.enabled or not supported:
            self.enabled = False
            return self._forward(image, *args, **kwargs)
        self._input.copy_(image)
        self._graph.replay()
        return self._output
