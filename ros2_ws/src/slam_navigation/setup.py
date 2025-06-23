from setuptools import setup
import os
from glob import glob

package_name = 'slam_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # Avoid using find_packages unless needed
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # If you have launch files in launch/
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ec24136',
    maintainer_email='ec24136@qmul.ac.uk',
    description='Simple SLAM navigation package for Webots robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slam_node = slam_navigation.slam_node:main',
        ],
    },
)
