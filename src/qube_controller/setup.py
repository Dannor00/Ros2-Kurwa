from setuptools import find_packages, setup

package_name = 'qube_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daniel-hatlem',
    maintainer_email='daniehn@ntnu.no',
    description='Simple PID controller for the Quanser Qube using ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_node = qube_controller.pid_node:main',
        ],
    },
)

