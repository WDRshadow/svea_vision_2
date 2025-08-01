from setuptools import find_packages, setup

package_name = 'svea_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'talker = svea_vision.talker:main',
        ],
    },
)
