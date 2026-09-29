import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'panda_camera_alignment'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='esjin',
    maintainer_email='lab.esjin4664@gmail.com',
    description='Aligns the RealSense camera frame with the Panda base frame via a fixed marker.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_align = panda_camera_alignment.aruco_align:main',
        ],
    },
)
