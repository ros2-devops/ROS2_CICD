from setuptools import find_packages, setup

package_name = 'sim_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sim.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ec24136',
    maintainer_email='ec24136@qmul.ac.uk',
    description='Minimal sim_demo package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'sim_demo = sim_demo.main:main',
    ],
}
,
)
