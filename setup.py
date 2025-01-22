from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gazebo_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf*'))),
        (os.path.join('share', package_name, 'sdf'), glob(os.path.join('sdf', '*.sdf*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='felix',
    maintainer_email='felix@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_publisher = gazebo_simulation.trajectory_publisher:main',
            'pid_controller_publisher = gazebo_simulation.pid_controller_publisher:main',
            'apply_wrench = gazebo_simulation.apply_wrench:main',
            'listener = gazebo_simulation.listener:main',
        ],
    },
)
