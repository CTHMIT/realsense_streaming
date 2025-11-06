import os
from glob import glob
from setuptools import setup

package_name = 'server_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config', 'dds'), 
            glob('config/dds/*.xml')),
        (os.path.join('share', package_name, 'rviz'), 
            glob('rviz/*.rviz')), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cthsu',
    maintainer_email='chuntsehsu@gmail.com',
    description='Server perception bringup package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)