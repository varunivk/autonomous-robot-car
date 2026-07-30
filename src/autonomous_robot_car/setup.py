from glob import glob
import os
from pathlib import Path

from setuptools import find_packages, setup


package_name = 'autonomous_robot_car'


def package_files(directory):
    """Return (install path, files) entries while preserving subdirectories."""
    entries = []
    for root, _, files in os.walk(directory):
        if '__pycache__' in Path(root).parts:
            continue
        if not files:
            continue
        paths = [os.path.join(root, name) for name in files]
        entries.append((
            os.path.join('share', package_name, root),
            paths,
        ))
    return entries


data_files = [
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]
for asset_directory in ('config', 'dashboard', 'launch', 'urdf', 'worlds'):
    data_files.extend(package_files(asset_directory))


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rafael',
    maintainer_email='rafael@example.com',
    description='Camera-only autonomous 4WD delivery robot simulation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_navigator = autonomous_robot_car.camera_navigator:main',
            'dashboard = autonomous_robot_car.dashboard_node:main',
        ],
    },
)
