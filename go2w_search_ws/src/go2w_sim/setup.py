from glob import glob
from setuptools import setup
setup(
    name='go2w_sim', version='0.1.0',
    packages=['go2w_sim', 'go2w_sim.nodes'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/go2w_sim']),
        ('share/go2w_sim', ['package.xml']),
        ('share/go2w_sim/launch', glob('launch/*.launch.py')),
        ('share/go2w_sim/config', glob('config/*.yaml')),
        ('share/go2w_sim/worlds', glob('worlds/*.world')),
        ('share/go2w_sim/urdf', glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='nhy', maintainer_email='nhy@todo.todo',
    description='Gazebo Classic 11 simulation of DOGS Nav2 stack',
    license='MIT', tests_require=['pytest'],
    entry_points={'console_scripts': [
        'motion_sdk_mock = go2w_sim.nodes.motion_sdk_mock:main',
    ]},
)
