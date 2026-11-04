# Copyright 2026 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for the 1 Euro filter and the frame clock."""

import numpy as np
import pytest

from dora_openarm_ker.main import FrameClock, KerPoseProcessor, OneEuroFilter

DT = 0.004


def test_constant_input_is_unchanged():
    """A signal at rest passes through exactly."""
    f = OneEuroFilter()
    x = np.array([0.1, -0.2, 0.3])
    for i in range(50):
        np.testing.assert_allclose(f.process(i * DT, x), x)


def test_step_converges_monotonically():
    """A step is approached from below without overshoot."""
    f = OneEuroFilter()
    f.process(0.0, [0.0])
    prev = 0.0
    for i in range(1, 500):
        y = f.process(i * DT, [1.0])[0]
        assert prev <= y <= 1.0
        prev = y
    assert prev == pytest.approx(1.0, abs=1e-6)


def test_fast_motion_lags_less_with_beta():
    """Raising beta reduces lag while moving."""
    lags = []
    for beta in (0.0, 5.0):
        f = OneEuroFilter(beta=beta)
        for i in range(200):
            t = i * DT
            y = f.process(t, [2.0 * t])[0]
        lags.append(2.0 * t - y)
    assert lags[1] < lags[0]


def test_non_increasing_time_holds_last_output():
    """A repeated timestamp returns the previous output."""
    f = OneEuroFilter()
    f.process(0.0, [0.0])
    y = f.process(DT, [1.0])
    np.testing.assert_allclose(f.process(DT, [5.0]), y)


def test_reset_passes_next_sample_through():
    """After reset the next sample is returned unfiltered."""
    f = OneEuroFilter()
    f.process(0.0, [0.0])
    f.process(DT, [0.0])
    f.reset()
    np.testing.assert_allclose(f.process(2 * DT, [1.0]), [1.0])


@pytest.mark.parametrize(
    "kwargs",
    [{"min_cutoff": 0.0}, {"d_cutoff": -1.0}, {"beta": -0.1}],
)
def test_invalid_parameters_are_rejected(kwargs):
    """Parameters that would break the alpha computation raise."""
    with pytest.raises(ValueError):
        OneEuroFilter(**kwargs)


def test_frame_clock_follows_counter():
    """Sample time advances by the frame gap, not by wall time."""
    clock = FrameClock(1000.0)
    assert clock.update(10) == (0.0, False)
    t, restarted = clock.update(14)
    assert t == pytest.approx(0.004)
    assert not restarted


def test_frame_clock_unwraps_uint32():
    """Wrapping past 2**32 counts as a small forward gap."""
    clock = FrameClock(1000.0)
    clock.update(0xFFFFFFFE)
    t, restarted = clock.update(2)
    assert t == pytest.approx(0.004)
    assert not restarted


def test_frame_clock_detects_restart():
    """A counter going back to zero is reported as a restart."""
    clock = FrameClock(1000.0)
    clock.update(5000)
    t, restarted = clock.update(0)
    assert restarted
    assert t > 0.0


def test_processor_leaves_grippers_unfiltered():
    """Only the 14 arm joints are smoothed."""
    processor = KerPoseProcessor(one_euro=OneEuroFilter())
    processor.process([0.0] * 16, t=0.0)
    right, left = processor.process([10.0] * 7 + [-30.0] + [10.0] * 7 + [30.0], t=DT)
    assert all(0.0 < v < np.deg2rad(10.0) for v in right[:7] + left[:7])
    unfiltered_right, unfiltered_left = KerPoseProcessor().process(
        [10.0] * 7 + [-30.0] + [10.0] * 7 + [30.0]
    )
    assert right[7] == unfiltered_right[7]
    assert left[7] == unfiltered_left[7]
