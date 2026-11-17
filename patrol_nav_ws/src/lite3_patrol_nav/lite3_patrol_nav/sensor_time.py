"""Read the ROS steady clock used by this Jetson's sensor publishers.

Foxy's rcutils uses CLOCK_MONOTONIC_RAW on Linux, whereas Python's
time.monotonic() uses CLOCK_MONOTONIC. Their origins/rates may differ.
Use the actual ROS API instead of assuming a Linux clock or estimating an
offset from message arrival. Message stamps are never rewritten here.
"""
from functools import lru_cache


@lru_cache(maxsize=1)
def _steady_clock():
    # Lazy imports keep the pure offline geometry/tools usable without ROS.
    from rclpy.clock import Clock, ClockType

    return Clock(clock_type=ClockType.STEADY_TIME)


def sensor_time():
    """Seconds in the same domain as RCL_STEADY_TIME sensor headers."""
    return _steady_clock().now().nanoseconds / 1_000_000_000
