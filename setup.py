import os
from glob import glob
from setuptools import setup

package_name = 'bag_unpack'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Jakub Chudziński',
    author_email='jakubach@gmail.com',
    maintainer='Jakub Chudziński',
    maintainer_email='jakubach@gmail.com',
    description='ROS2 utility package for excluding specific transformations and selecting topics for output',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bag_unpack = bag_unpack.bag_unpack:main',
        ],
    },
    package_data={
        package_name: [
            'config/*.yaml',
            'launch/*.py', 
        ],
    },
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml')))
    ]
)
