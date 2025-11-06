import os
from glob import glob
from setuptools import setup

package_name = 'server_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config', 'vslam'), 
            glob('config/vslam/*.yaml')),
        (os.path.join('share', package_name, 'config', 'nvblox'), 
            glob('config/nvblox/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cthsu',
    maintainer_email='chuntsehsu@gmail.com',
    description='Perception nodes and configs for the server',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)