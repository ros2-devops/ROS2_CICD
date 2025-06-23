from setuptools import setup

package_name = 'cpu_hog_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    py_modules=['hog'],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='A ROS 2 node that simulates CPU stress',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hog = cpu_hog_controller.cpu_hog_controller:main',
        ],
    },
)
