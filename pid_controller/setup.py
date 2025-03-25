from setuptools import setup
import os
from glob import glob

package_name = 'pid_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        # Inkluder launch-filer
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Inkluder config-filer
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Hatlem',
    maintainer_email='you@example.com',
    description='PID controller node using custom ROS2 service and reference input',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_controller_node = pid_controller.pid_controller_node:main',
            'reference_input_node = pid_controller.reference_input_node:main',
        ],
    },
)

