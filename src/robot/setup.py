import os
from glob import glob
from setuptools import setup

package_name = 'robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'diagnostics', 'launch'), 
            glob('diagnostics/launch/*.launch.py')),
        (os.path.join('share', package_name, 'diagnostics','config'), 
            glob('diagnostics/config/*.yaml')),
        (os.path.join('share', package_name, 'description', 'launch'), 
            glob('description/launch/*.launch.py')),
        (os.path.join('share', package_name, 'description', 'urdf'), 
            glob('description/urdf/*.*')), # .urdf and .xacro
        (os.path.join('share', package_name, 'description', 'xrdf'), 
            glob('description/xrdf/*.xrdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cthsu',
    maintainer_email='chuntsehsu@gmail.com',
    description='Diagnostics aggregator for the robot',
    license='Wistron Public View-Only License (WPVOL) v1.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)