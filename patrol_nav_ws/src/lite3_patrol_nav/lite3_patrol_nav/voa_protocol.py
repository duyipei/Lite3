"""ROS-independent fail-closed correlation for factory VOA corrections."""
import math
import uuid


class VoaExchange:
    def __init__(self, timeout=0.25):
        self.timeout = timeout
        self.prefix = 'patrol_voa_' + uuid.uuid4().hex + '_'
        self.sequence = 0
        self.pending = {}
        self.last = None
        self.validated_time = None
        self.last_sequence = -1

    def reset(self):
        # A zero-speed probe is independent of the navigation goal. Retain
        # only those outstanding probes across ARM/goal transitions so their
        # fresh responses can still prove sensor health. Discard every pending
        # motion correction; it must never be reused for another goal.
        self.pending = {k: v for k, v in self.pending.items()
                        if v[1] == (0., 0., 0.)}
        self.last = None

    def issue(self, velocity, now):
        self.pending = {k: v for k, v in self.pending.items() if now-v[0] <= self.timeout}
        self.sequence += 1
        key = self.prefix + str(self.sequence)
        self.pending[key] = (now, tuple(velocity), self.sequence)
        return key

    def accept(self, key, velocity, now):
        request = self.pending.pop(key, None)
        if request is None:
            return False
        sent, original, sequence = request
        if now-sent > self.timeout or now < sent or sequence <= self.last_sequence:
            return False
        vx, vy, w = velocity
        if not all(math.isfinite(v) for v in velocity):
            self.last = None
            self.validated_time = None
            return False
        if not (0 <= vx <= 0.15 and vy == 0 and abs(w) <= 0.4):
            self.last = None
            self.validated_time = None
            return False
        if original == (0., 0., 0.) and tuple(velocity) != original:
            self.last = None
            self.validated_time = None
            return False
        self.last_sequence = sequence
        # Freshness is measured from request, NOT from delayed response arrival.
        self.last = (sent, original, tuple(velocity))
        self.validated_time = sent
        return True

    def healthy(self, now):
        return self.validated_time is not None and 0 <= now-self.validated_time <= self.timeout

    def corrected(self, original, now):
        if not self.healthy(now) or self.last is None or self.last[1] != tuple(original):
            return (0., 0., 0.)
        if tuple(original) == (0., 0., 0.):
            return (0., 0., 0.)
        return self.last[2]


def braking_correction(original, proposal):
    """Accept only same-path slowing; Nav2 alone chooses steering.

    VOA has a near-field depth map, not Nav2's static-map footprint costs.
    Its alternate steering candidates therefore cannot bypass that planner.
    A scalar applied to BOTH velocities preserves the planned curvature.
    """
    zero = (0., 0., 0.)
    if len(original) != 3 or len(proposal) != 3:
        return zero
    if not all(math.isfinite(v) for v in (*original, *proposal)):
        return zero
    if original[1] != 0 or proposal[1] != 0 or original[0] < 0:
        return zero
    if tuple(proposal) == zero:
        return zero
    active = [i for i in (0, 2) if abs(original[i]) > 1.e-9]
    if not active:
        return zero
    scale = proposal[active[0]] / original[active[0]]
    if not 0 <= scale <= 1:
        return zero
    expected = tuple(v * scale for v in original)
    if any(abs(a - b) > 1.e-6 for a, b in zip(expected, proposal)):
        return zero
    return expected
