import os
from glob import glob
from setuptools import setup

package_name = 'go2_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'launch', 'launch_ros', 'ament_index_python'],
    zip_safe=True,
    maintainer='brian',
    maintainer_email='you@example.com',
    description='Bringup for Go2W + RealSense + Aruco + arm + nav nodes',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)


