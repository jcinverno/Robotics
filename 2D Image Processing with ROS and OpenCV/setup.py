from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'trsa_lab1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'video'), glob('video/*.mov')),
        (os.path.join('share', package_name, 'calibration/images'), glob('calibration/images/*')),
        (os.path.join('share', package_name, 'calibration'), glob('calibration/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='trsa2024',
    maintainer_email='trsa2024@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_calibration_pub = trsa_lab1.camera_calibration_pub:main',
            'camera_driver = trsa_lab1.camera_driver:main',
            'image_rectifier = trsa_lab1.image_rectifier:main',
            'image_processing = trsa_lab1.image_processing:main', 
            'my_camera = trsa_lab1.my_camera:main', 
            'my_filter = trsa_lab1.my_filter:main', 
            'image_reader = trsa_lab1.image_reader:main',
        ],
    },
)
