from setuptools import setup

package_name = 'pid_velocity_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'pid_velocity_node = pid_velocity_controller.pid_velocity_node:main',
        ],
    },
)
