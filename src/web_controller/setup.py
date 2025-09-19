import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'web_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # --- THIS IS THE CRITICAL FIX ---
        (os.path.join('lib', package_name), ['scripts/web_api_server']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Web API controller for Amadeus robot',
    license='MIT',
    tests_require=['pytest'],
)