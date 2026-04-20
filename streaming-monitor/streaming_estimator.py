class StreamingEstimator:
    """
    Estimate a stable streaming bitrate from bursty switch-port samples.

    Raw SNMP throughput samples are a poor direct proxy for user-visible stream
    quality: media playback tends to arrive in short bursts, then go quiet while
    the player drains its buffer. A plain moving average reacts too strongly to
    those spikes and troughs, so the reported bitrate jumps around more than the
    underlying stream quality actually does.

    This estimator uses an exponential moving average (EMA). Each observation
    contributes an ``alpha`` fraction of the new estimate while the previous
    estimate keeps the remaining ``1 - alpha`` weight. That gives us a simple
    one-state filter that is cheap, stable, and still able to follow real
    sustained changes in bitrate.

    Samples below ``floor`` are treated as zero so tiny background chatter does
    not look like real playback. After smoothing, estimates below
    ``minimum_output`` are clamped to ``0.0`` so the published value cleanly
    drops to zero instead of hovering around fractional noise.

    Tuning tradeoffs:
    - Higher ``alpha`` tracks changes faster, but makes the output follow burst
      noise more closely and therefore look less stable.
    - Lower ``alpha`` produces a calmer number, but reacts more slowly when the
      actual stream bitrate genuinely changes.
    - Higher ``floor`` ignores more low-level traffic, but can hide very low
      bitrate playback.
    - Higher ``minimum_output`` makes the estimator snap to zero more readily,
      while lower values allow a longer low-level tail before the output clears.
    """

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
