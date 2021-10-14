import os
from glob import glob
from setuptools import setup

package_name = 'cka'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
	    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='local',
    maintainer_email='ch.adolph@ostfalia.de',
    description='Turtlebot 3 - WS2122',
    license='No license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		    'zigzag = cka.zigzag:main'
        ],
    },
)
