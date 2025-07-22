from setuptools import setup

package_name = 'slam_node_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ec24136',
    maintainer_email='ec24136@qmul.ac.uk',
    description='Simulates SLAM navigation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'slam_node = slam_node_controller.slam_node:main',
        ],
    },
)
