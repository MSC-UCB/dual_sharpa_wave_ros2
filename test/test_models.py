from pathlib import Path
import importlib.util
import xml.etree.ElementTree as ET

import pytest

from dual_sharpa_wave.joint_names import joint_names


ROOT = Path(__file__).resolve().parents[1]
_SPEC = importlib.util.spec_from_file_location(
    'dual_sharpa_launch', ROOT / 'launch' / 'dual_sharpa.launch.py'
)
launch_file = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(launch_file)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_model_joint_order_and_meshes(side):
    root = ET.parse(ROOT / 'urdf' / f'{side}_sharpa_wave_with_flange.urdf').getroot()
    movable = [j for j in root.findall('joint') if j.get('type') != 'fixed']
    assert tuple(j.get('name') for j in movable) == joint_names(side)
    assert all(j.get('type') == 'revolute' for j in movable)
    assert not list(root.iter('mimic'))
    for mesh in root.iter('mesh'):
        uri = mesh.get('filename')
        assert uri.startswith('package://dual_sharpa_wave/')
        assert (ROOT / uri.removeprefix('package://dual_sharpa_wave/')).is_file()


def test_preview_has_unique_child_frames_and_common_root():
    children = []
    for side in ('left', 'right'):
        root = ET.fromstring(launch_file._preview_description(ROOT, side))
        links = {link.get('name') for link in root.findall('link')}
        side_children = [j.find('child').get('link') for j in root.findall('joint')]
        assert links - set(side_children) == {'preview_world'}
        assert len(side_children) == len(set(side_children))
        children.extend(side_children)
    assert len(children) == len(set(children))
