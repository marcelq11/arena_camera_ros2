from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sign_recognition_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/msg', ['msg/CameraSettings.msg']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='opszalek',
    maintainer_email='dawidc4@hotmail.com',
    description='A package for sign recognition',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sign_recognition_node_main = sign_recognition_node.sign_recognition_node_main:main',
        ],
    },
)
