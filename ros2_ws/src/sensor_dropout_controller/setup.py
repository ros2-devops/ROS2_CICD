from setuptools import setup

package_name = 'sensor_dropout_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ec24136',
    maintainer_email='ec24136@qmul.ac.uk',
    description='Simulates sensor dropout',
    license='MIT',
    entry_points={
        'console_scripts': [
            'sensor_dropout = sensor_dropout_controller.sensor_dropout:main',
        ],
    },
)
