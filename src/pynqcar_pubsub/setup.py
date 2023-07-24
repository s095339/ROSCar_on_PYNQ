from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'pynqcar_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xilinx',
    maintainer_email='xilinx@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Driver = pynqcar_pubsub.ctrl_pub:main',
            'Car = pynqcar_pubsub.pynqcar_sub:main',
            'listen = pynqcar_pubsub.speed:main'
        ],
    },
)
