#!/usr/bin/env python3

import argparse
from collections import deque

from streaming_estimator import StreamingEstimator


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("capture_file")
    parser.add_argument("--interval", type=float, default=5.0)
    return parser


def iter_capture_samples(path: str, interval_s: float):
    bucket_megabits = 0.0
    bucket_duration_s = 0.0

    with open(path, encoding="utf-8") as file:
        for line in file:
            stripped = line.strip()
            if not stripped:
                continue

            observed_mbps = float(stripped)
            bucket_megabits += observed_mbps
            bucket_duration_s += 1.0

            if bucket_duration_s >= interval_s:
                yield bucket_duration_s, bucket_megabits / bucket_duration_s, bucket_megabits
                bucket_megabits = 0.0
                bucket_duration_s = 0.0

    if bucket_duration_s > 0:
        yield bucket_duration_s, bucket_megabits / bucket_duration_s, bucket_megabits


def main() -> None:
    args = build_parser().parse_args()
    running_average_window = 10
    running_average_samples: deque[float] = deque()
    running_average_sum = 0.0
    estimator = StreamingEstimator()

    for _duration_s, observed_mbps, _observed_amount in iter_capture_samples(args.capture_file, args.interval):
        estimate = estimator.observe(observed_mbps)

        running_average_samples.append(observed_mbps)
        running_average_sum += observed_mbps
        if len(running_average_samples) > running_average_window:
            running_average_sum -= running_average_samples.popleft()
        running_average = running_average_sum / len(running_average_samples)

        print(
            f"obs={observed_mbps:8.3f} avg={running_average:8.3f} est={estimate:8.3f}"
        )


if __name__ == "__main__":
    main()
