class StreamingEstimator:
    def __init__(self, *, alpha: float = 0.10, floor: float = 0.20, minimum_output: float = 1.0) -> None:
        self.alpha = alpha
        self.floor = floor
        self.minimum_output = minimum_output
        self._estimate = None

    def reset(self) -> None:
        self._estimate = None

    def observe(self, observed_value: float) -> float:
        sample = 0.0 if observed_value < self.floor else observed_value
        if self._estimate is None:
            self._estimate = sample
        else:
            self._estimate = self.alpha * sample + (1.0 - self.alpha) * self._estimate
        if self._estimate < self.minimum_output:
            return 0.0
        return self._estimate
