import os

from setuptools import find_packages, setup

package_name = 'my_people_nvblox_bringup'


def collect_data_files(directory):
    """Install every file while preserving its directory structure."""
    data_files = []

    if not os.path.isdir(directory):
        return data_files

    for current_path, _, filenames in os.walk(directory):
        if not filenames:
            continue

        source_files = [
            os.path.join(current_path, filename)
            for filename in filenames
        ]

        relative_path = os.path.relpath(current_path, '.')
        install_path = os.path.join(
            'share',
            package_name,
            relative_path,
        )

        data_files.append((install_path, source_files))

    return data_files


data_files = [
    (
        'share/ament_index/resource_index/packages',
        ['resource/' + package_name],
    ),
    (
        'share/' + package_name,
        ['package.xml'],
    ),
]

data_files += collect_data_files('launch')
data_files += collect_data_files('config')


setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='esjin',
    maintainer_email='esjin@todo.todo',
    description='Custom NVBlox people segmentation bringup for Isaac Sim',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'closest_panda_human_voxels = '
            'my_people_nvblox_bringup.closest_panda_human_voxels:main',
            'panda_voxel_classifier = '
            'my_people_nvblox_bringup.panda_voxel_classifier:main',
            'plot_filtered_distance = '
            'my_people_nvblox_bringup.plot_filtered_distance:main',
            'plot_distances = '
            'my_people_nvblox_bringup.plot_filtered_distance:main_both',
        ],
    },
)
