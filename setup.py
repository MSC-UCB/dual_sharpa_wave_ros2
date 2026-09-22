from pathlib import Path

from setuptools import find_packages, setup


package_name = 'dual_sharpa_wave'
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml', 'README.md', 'LICENSE']),
]
for directory in ('config', 'launch', 'urdf', 'rviz', 'third_party', 'docs'):
    for path in sorted(Path(directory).rglob('*')):
        if path.is_file() and '__pycache__' not in path.parts:
            data_files.append((str(Path('share') / package_name / path.parent), [str(path)]))

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Howard',
    maintainer_email='howard@example.com',
    description='Independent Sharpa Wave hand interfaces and offline mock backend',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'hand_node = dual_sharpa_wave.hand_node:main',
        'gui_control.py = dual_sharpa_wave.gui_control:main',
        'sine_control.py = dual_sharpa_wave.wave_control:sine_main',
        'step_control.py = dual_sharpa_wave.wave_control:step_main',
    ]},
)
