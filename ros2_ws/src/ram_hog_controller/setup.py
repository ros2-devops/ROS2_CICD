from setuptools import setup

package_name = 'ram_hog_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ec24136',
    maintainer_email='ec24136@qmul.ac.uk',
    description='Simulates RAM stress on a robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ram_hog = ram_hog_controller.ram_hog:main',
        ],
    },
)
