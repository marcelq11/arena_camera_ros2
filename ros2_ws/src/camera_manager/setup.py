from setuptools import find_packages, setup
from glob import glob

package_name = 'camera_manager'

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
    maintainer='marcel',
    maintainer_email='marceq38@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'camera_setting = camera_manager.camera_setting:main',
        'camera_manager = camera_manager.camera_manager:main',
        'camera_preview = camera_manager.camera_preview:main',
        ],
    },
)
