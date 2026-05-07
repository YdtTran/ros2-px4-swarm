from setuptools import find_packages, setup

package_name = 'uav_swarm_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/swarm_demo.launch.py']),
        ('share/' + package_name + '/config', ['config/swarm_demo.rviz']),
    ],
    install_requires=['setuptools', 'pathfinding', 'pyrvo'],
    zip_safe=True,
    maintainer='pc',
    maintainer_email='hieu.ngminh98@gmail.com',
    description='3-UAV Leader-Follower swarm demo with A* + ORCA collision avoidance',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'swarm_controller = uav_swarm_demo.nodes.swarm_controller_node:main',
            'px4_swarm        = uav_swarm_demo.nodes.px4_swarm_node:main',
        ],
    },
)
