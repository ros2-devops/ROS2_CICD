from setuptools import setup

package_name = 'io_stall_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ec24136',
    maintainer_email='ec24136@qmul.ac.uk',
    description='Simulates I/O stalls in a robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'io_stall = io_stall_controller.io_stall:main',
        ],
    },
)
