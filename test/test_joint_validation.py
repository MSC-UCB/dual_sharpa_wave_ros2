import pytest

from dual_sharpa_wave.joint_names import joint_names
from dual_sharpa_wave.joint_validation import canonical_positions


@pytest.mark.parametrize('side', ['left', 'right'])
def test_reorder(side):
    names = joint_names(side)
    positions = [i / 100 for i in range(22)]
    assert canonical_positions([], positions, names) == positions
    assert canonical_positions(names[::-1], positions[::-1], names) == positions


@pytest.mark.parametrize('kind', ['short', 'long', 'duplicate', 'unknown', 'other_side'])
def test_reject_bad_names(kind):
    names = list(joint_names('left'))
    if kind == 'short':
        names.pop()
    elif kind == 'long':
        names.append('extra')
    elif kind == 'duplicate':
        names[0] = names[1]
    elif kind == 'unknown':
        names[0] = 'not_a_joint'
    else:
        names = list(joint_names('right'))
    with pytest.raises(ValueError):
        canonical_positions(names, [0.0]*22, joint_names('left'))


@pytest.mark.parametrize('value', [True, '0.1', None])
def test_reject_non_numeric(value):
    with pytest.raises(ValueError):
        canonical_positions([], [value] * 22, joint_names('left'))
