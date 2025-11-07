import os
from glob import glob
from setuptools import setup

package_name = 'server'

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
        (os.path.join('share', package_name, 'bringup', 'rviz'), 
            glob('bringup/rviz/*.rviz')), 
        (os.path.join('share', package_name, 'perception', 'config', 'vslam'), 
            glob('perception/config/vslam/*.yaml')),
        (os.path.join('share', package_name, 'perception', 'config', 'nvblox'), 
            glob('perception/config/nvblox/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cthsu',
    maintainer_email='chuntsehsu@gmail.com',
    description='Perception nodes and configs for the server',
    license='Wistron Public View-Only License (WPVOL) v1.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)