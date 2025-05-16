from setuptools import setup
import os
from glob import glob

package_name = 'pure_pursuit_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=[
        'setuptools',
        'numpy',
        'tf_transformations',
    ],
    zip_safe=True,
    maintainer='Junaet Mahbub',
    maintainer_email='junaetmahbub4@gmail.com',
    description='ROS2 pure pursuit lateral controller',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pure_pursuit_node = pure_pursuit_controller.pure_pursuit_node:main',
        ],
    },
    data_files=[
        # package manifest
        ('share/' + package_name, ['package.xml']),
        # install the launch file
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # install waypoints CSV
        (os.path.join('share', package_name, 'waypoints'),
         glob('pure_pursuit_controller/waypoints/*.csv')),
    ],
)
