from pathlib import Path
import math

import pytest

from dual_sharpa_wave.control_model import (
    SIDES, Waveform, WaveSequence, axis_indices, bounded_positions, load_limits,
)

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def limits():
    return load_limits(ROOT)


@pytest.fixture
def baseline():
    return {'left': [0.02] * 22, 'right': [0.03] * 22}


def test_sine_endpoints_and_step_plateaus():
    sine = Waveform('sine', 0.1, 1.0, 2)
    assert [sine.offset(t) for t in (0, 0.25, 0.75, 2, 10)] == pytest.approx([0, 0.1, -0.1, 0, 0])
    step = Waveform('step', -0.1, cycles=2, base_seconds=1, high_seconds=2)
    assert [step.offset(t) for t in (0, 1, 2.9, 3, 4, 5, 7, 8)] == [0, -0.1, -0.1, 0, 0, -0.1, 0, 0]


def test_pair_offsets_unselected_preserved_and_baseline_not_aliased(limits, baseline):
    sequence = WaveSequence(baseline, limits, (0, 5), Waveform('sine', 0.1, 1.0, 1))
    target = sequence.sample(0.25, baseline)
    assert target['left'][0] == pytest.approx(0.12)
    assert target['right'][0] == pytest.approx(0.13)
    for side in SIDES:
        assert target[side][1:] == baseline[side][1:]
    target['left'][0] = 999
    baseline['right'][0] = 999
    assert sequence.baseline['left'][0] == 0.02
    assert sequence.baseline['right'][0] == 0.03


def test_waits_for_both_hands_then_advances_and_finishes(limits, baseline):
    sequence = WaveSequence(baseline, limits, (0, 5), Waveform('sine', 0.1, 1.0, 1), gap=0.1)
    assert sequence.sample(1, baseline) == baseline
    away = {s: q.copy() for s, q in baseline.items()}
    away['right'][0] += 0.1
    assert sequence.sample(1.2, away) == baseline
    assert sequence.index == 0
    sequence.sample(1.3, baseline)
    assert sequence.index == 1
    assert sequence.sample(1.55, baseline)['left'][5] == pytest.approx(0.12)
    assert sequence.sample(2.4, baseline) == baseline
    assert sequence.sample(2.6, baseline) is None
    assert sequence.done


def test_settle_timeout_and_repetition(limits, baseline):
    seq = WaveSequence(baseline, limits, (0,), Waveform('step', cycles=1), repeat=2, settle_timeout=1)
    assert len(seq.axes) == 2
    seq.sample(3, baseline)
    away = {s: [q + 0.1 for q in baseline[s]] for s in SIDES}
    with pytest.raises(TimeoutError):
        seq.sample(4, away)


@pytest.mark.parametrize('axes', ['', 'bogus', 'thumb_IP,thumb_IP', 'left_thumb_IP'])
def test_axis_list_rejects_unknown_duplicates_and_side_prefix(axes):
    with pytest.raises(ValueError):
        axis_indices(axes)


def test_limits_check_entire_trajectory_on_both_sides(limits, baseline):
    # A sine centered at zero cannot bend a PIP below its lower limit.
    baseline['left'][7] = 0.2
    baseline['right'][7] = 0
    with pytest.raises(ValueError, match='right_index_PIP trajectory'):
        WaveSequence(baseline, limits, (7,), Waveform('sine'))
    WaveSequence(baseline, limits, (7,), Waveform('step'))
    with pytest.raises(ValueError):
        bounded_positions('left', [math.nan] * 22, limits)


@pytest.mark.parametrize('kwargs', [{'amplitude': float('nan')}, {'frequency': 0}, {'cycles': 0},
                                    {'base_seconds': -1}, {'high_seconds': float('inf')}])
def test_bad_waveform_parameters(kwargs):
    with pytest.raises(ValueError):
        Waveform('sine', **kwargs)
