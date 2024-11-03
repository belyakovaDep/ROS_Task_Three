from setuptools import find_packages, setup
from glob import glob

package_name = 'service_full_name'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/srv', glob('srv/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='belyakovaDep',
    maintainer_email='d.belyakova1@g.nsu.ru',
    description='Simple messages and services v2',
    license='Apache-2.0 license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service_name = service_full_name.service_name:main',
            'client_name = service_full_name.client_name:main',
        ],
    },
)
