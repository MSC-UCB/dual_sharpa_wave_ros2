"""Canonical URDF order, checked against SDK 5.0.10.6 sample indices 0..21.

Model revision: 0d19cac602f46456b819e4b6a2c09a74982c9a3e, with_flange variants.
SDK calls thumb joint 4 DIP; the official URDF names the same joint thumb_IP.
"""

JOINT_SUFFIXES = (
    'thumb_CMC_FE', 'thumb_CMC_AA', 'thumb_MCP_FE', 'thumb_MCP_AA', 'thumb_IP',
    'index_MCP_FE', 'index_MCP_AA', 'index_PIP', 'index_DIP',
    'middle_MCP_FE', 'middle_MCP_AA', 'middle_PIP', 'middle_DIP',
    'ring_MCP_FE', 'ring_MCP_AA', 'ring_PIP', 'ring_DIP',
    'pinky_CMC', 'pinky_MCP_FE', 'pinky_MCP_AA', 'pinky_PIP', 'pinky_DIP',
)
JOINT_COUNT = len(JOINT_SUFFIXES)
# sdk[i] maps to canonical[SDK_TO_URDF_INDEX[i]]. Both checked models use identity.
SDK_TO_URDF_INDEX = tuple(range(JOINT_COUNT))
URDF_TO_SDK_INDEX = tuple(SDK_TO_URDF_INDEX.index(i) for i in range(JOINT_COUNT))


def joint_names(side: str) -> tuple[str, ...]:
    if side not in ('left', 'right'):
        raise ValueError("side must be 'left' or 'right'")
    return tuple(f'{side}_{suffix}' for suffix in JOINT_SUFFIXES)
