import os
from glob import glob
from setuptools import setup

package_name = 'arm_ik_service'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'ik_service_node = arm_ik_service.ik_service_node:main',
            'planning_scene_node = arm_ik_service.planning_scene_node:main',
            'reachability_heatmap = arm_ik_service.reachability_heatmap:main',
            'coverage_planner = arm_ik_service.coverage_planner:main',
            'wiping_controller = arm_ik_service.wiping_controller:main',
            'trajectory_executor = arm_ik_service.trajectory_executor:main',
            'tuck_pose_sender = arm_ik_service.tuck_pose_sender:main',
        ],
    },
)