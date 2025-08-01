from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'svea_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jackdaw',
    maintainer_email='wdrshadow@gmail.com',
    description='The svea vision package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detect = svea_vision.scripts.aruco_detect:main',
            'static_image_publisher = svea_vision.scripts.static_image_publisher:main',
            'object_detect = svea_vision.scripts.object_detect:main',
            'object_pose = svea_vision.scripts.object_pose:main',
            'person_state_estimation = svea_vision.scripts.person_state_estimation:main',
            'sidewalk_mapper = svea_vision.scripts.sidewalk_mapper:main',
            'detection_splitter = svea_vision.scripts.detection_splitter:main',
            'pedestrian_flow_estimate = svea_vision.scripts.pedestrian_flow_estimate:main',
            'segment_anything = svea_vision.scripts.segment_anything:main',
        ],
    },
)
