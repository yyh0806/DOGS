from setuptools import setup, find_packages

package_name = 'go2w_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nhy',
    maintainer_email='nhy@todo.todo',
    description='Go2W SDK to ROS2 bridge node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = go2w_bridge.bridge_node:main',
            'sport_client = go2w_bridge.sport_client:main',
            'nx_sensor_node = go2w_bridge.nx_sensor_node:main',
            'nx_motion_node = go2w_bridge.nx_motion_node:main',
            'nx_panel_bridge = go2w_bridge.nx_panel_bridge:main',
        ],
    },
)
