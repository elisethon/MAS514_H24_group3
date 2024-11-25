from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robin_SAS'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=[
        'robin_SAS.teleop_to_serial',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	(os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='uiabot',
    maintainer_email='uiabot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	   'teleop_to_serial = robin_SAS.teleop_to_serial:main',
        ],
    },
)
