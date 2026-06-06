from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'lane_length_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jhp',
    maintainer_email='jhp@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'detection_node = lane_length_pkg.lane_detection_node:main',
        'integration_node = lane_length_pkg.lane_integration_node:main',
        'control_node = lane_length_pkg.lane_follow_control_node:main',
        'decision_node = lane_length_pkg.lane_decision_node:main',
        'viewer_node = lane_length_pkg.lane_viewer_node:main',
        'lane_detection_node = lane_length_pkg.lane_detection_node:main',
        'lane_integration_node = lane_length_pkg.lane_integration_node:main',
        'lane_decision_node = lane_length_pkg.lane_decision_node:main',
        'lane_follow_control_node = lane_length_pkg.lane_follow_control_node:main',
        'lane_viewer_node = lane_length_pkg.lane_viewer_node:main',
        'scene_fusion_node = lane_length_pkg.scene_fusion_node:main',
        'drive_mode_node = lane_length_pkg.drive_mode_node:main',
        'behavior_plan_node = lane_length_pkg.behavior_plan_node:main',
        'reverse_astar_planner_node = lane_length_pkg.reverse_astar_planner_node:main',
        'path_track_node = lane_length_pkg.path_track_node:main',
        'lane_follow_eval = lane_length_pkg.lane_follow_eval_node:main',
        'cmd_mux_node = lane_length_pkg.cmd_mux_node:main',
        'safety_filter_node = lane_length_pkg.safety_filter_node:main',
        ],
    },
)
