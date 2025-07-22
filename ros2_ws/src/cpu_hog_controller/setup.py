from setuptools import setup

package_name = 'cpu_hog_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your-name',
    maintainer_email='your-email',
    description='CPU hog controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hog = cpu_hog_controller.hog:main',
        ],
    },
)
