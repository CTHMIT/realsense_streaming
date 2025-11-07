import os
from glob import glob
from setuptools import setup

package_name = 'client'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'bringup', 'launch'), 
            glob('bringup/launch/*.launch.py')),
        (os.path.join('share', package_name, 'bringup', 'config', 'dds'), 
            glob('bringup/config/dds/*.xml')),
        (os.path.join('share', package_name, 'bringup', 'systemd'), 
            glob('bringup/systemd/*.service')),
        (os.path.join('share', package_name, 'sensors', 'config'), 
            glob('sensors/config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cthsu',
    maintainer_email='chuntsehsu@gmail.com',
    description='Orin client bringup package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)