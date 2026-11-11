import os
from glob import glob
from setuptools import setup

package_name = 'search_and_rescue_comp_26'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DAT160 gruppe 8',
    maintainer_email='todo@todo.todo',
    description='Search and rescue robot team for the DAT160 semester project',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'marker_reporter = search_and_rescue_comp_26.marker_reporter:main',
            'navigator = search_and_rescue_comp_26.navigator:main',
            'mission = search_and_rescue_comp_26.mission:main',
            'wall_colour = search_and_rescue_comp_26.wall_colour:main',
        ],
    },
)
