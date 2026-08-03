from setuptools import find_packages, setup

package_name = 'delivery_robot_hw'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='varunivkulkarni',
    maintainer_email='kulkarnivaruni151@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motor_driver_node = delivery_robot_hw.motor_driver_node:main',
            'encoder_node = delivery_robot_hw.encoder_node:main',
            'camera_node = delivery_robot_hw.camera_node:main',
        ],
    },
)
