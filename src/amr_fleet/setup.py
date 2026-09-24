from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'amr_fleet'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fleet Engineering Team',
    maintainer_email='robotics-fleet@antigravity.io',
    description='Decentralized Multi-AMR Coordination and Collision-Avoidance Framework',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_publisher = amr_fleet.state_publisher:main',
            'intent_publisher = amr_fleet.intent_publisher:main',
            'global_planner = amr_fleet.global_planner:main',
            'local_planner_orca = amr_fleet.local_planner_orca:main',
            'conflict_resolver = amr_fleet.conflict_resolver:main',
            'task_allocator = amr_fleet.task_allocator:main',
            'battery_monitor = amr_fleet.battery_monitor:main',
        ],
    },
)
