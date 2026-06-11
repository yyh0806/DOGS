from setuptools import setup, find_packages

package_name = 'go2w_orchestrator'

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
    description='Go2W task orchestration with VLM',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'orchestrator_node = go2w_orchestrator.orchestrator_node:main',
        ],
    },
)
