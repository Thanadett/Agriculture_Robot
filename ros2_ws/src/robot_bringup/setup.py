from setuptools import setup
from glob import glob

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('lib/' + package_name, ['robot_bringup/run_camera.sh']),
    ],
    install_requires=[
        'setuptools',
        'pyserial',
        'flask',
        'opencv-python'
    ],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='Joystick-only teleop with serial bridge to ESP32',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'joystick_teleop = robot_bringup.joystick_teleop:main',
            'serial_bridge = robot_bringup.serial_bridge:main',
            'camera_stream = robot_bringup.camera_stream:main',
        ],
    },
    python_requires='>=3.12',
)
