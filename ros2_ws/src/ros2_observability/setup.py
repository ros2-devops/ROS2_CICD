from setuptools import setup

package_name = 'ros2_observability'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ec24136',
    maintainer_email='ec24136@qmul.ac.uk',
    description='Simple CPU/Memory metrics collector for Webots CI pipeline',
    license='MIT',
    entry_points={
        'console_scripts': [
            'metrics_collector = ros2_observability.metrics_collector:main'
        ],
    },
)

