import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'go2w_web'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'static'),
            glob('go2w_web/static/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nhy',
    maintainer_email='nhy@todo.todo',
    description='Go2W web interface bridge',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_bridge_node = go2w_web.web_bridge_node:main',
        ],
    },
)
