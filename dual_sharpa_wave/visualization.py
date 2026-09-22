"""Optional display-only model preparation, independent of either hand backend."""

from pathlib import Path
import xml.etree.ElementTree as ET


def preview_description(share: Path, side: str) -> str:
    if side not in ('left', 'right'):
        raise ValueError('side must be left or right')
    root = ET.parse(share / 'urdf' / f'{side}_sharpa_wave_with_flange.urdf').getroot()
    ET.SubElement(root, 'link', name='preview_world')
    joint = ET.SubElement(root, 'joint', name=f'preview_{side}_mount', type='fixed')
    ET.SubElement(joint, 'parent', link='preview_world')
    ET.SubElement(joint, 'child', link=f'{side}_hand_flange')
    ET.SubElement(joint, 'origin', xyz=f'0 {0.15 if side == "left" else -0.15} 0', rpy='0 0 0')
    return ET.tostring(root, encoding='unicode')
