from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'route_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team 20',
    maintainer_email='go29hir@tum.de',
    description='Route loading, progress tracking, and local path publication.',
    license='Proprietary',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'route_manager_node = route_manager.route_manager_node:main',
            'waypoint_recorder = '
            'route_manager.interactive_waypoint_recorder:main',
        ],
    },
)
