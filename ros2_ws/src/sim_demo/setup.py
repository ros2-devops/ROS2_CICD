from setuptools import setup
import os
from glob import glob

package_name = 'sim_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # ROS 2 ament indexing
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        
        # Include worlds
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.wbt')),
        
        # Include controller Python files if needed (optional)
        (os.path.join('share', package_name, 'controllers'), glob('controllers/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ec24136',
    maintainer_email='ec24136@qmul.ac.uk',
    description='Simulation demo package with Webots and ROS 2 integration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_demo = sim_demo.main:main',
        ],
    },
)
