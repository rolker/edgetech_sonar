from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'edgetech_sonar'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Roland Arsenault',
    maintainer_email='roland@ccom.unh.edu',
    description='Driver for Edgetech sidescan sonar',
    license='BSD-2-Clause',
    entry_points={
        'console_scripts': [
          'edgetech_sonar = edgetech_sonar.edgetech_sonar:main',
        ],
    },
)

