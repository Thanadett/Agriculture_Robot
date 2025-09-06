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
        ('share/' + package_name + '/templates', glob('templates/*')),
        ('share/' + package_name + '/static', glob('static/*')),
    ],
    install_requires=[
        'setuptools',
        'pyserial',
        'flask',
        'opencv-python'
        'numpy',
        'pyturbojpeg',
    ],
    zip_safe=True,
    maintainer='Thanadet Thanapremin',
    maintainer_email='n.thanadett@gmail.com',
    description='391 Project',
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
