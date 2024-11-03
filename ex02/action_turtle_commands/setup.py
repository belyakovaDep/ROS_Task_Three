from setuptools import find_packages, setup
from glob import glob

package_name = 'action_turtle_commands'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/action', glob('action/*.action')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='belyakovaDep',
    maintainer_email='d.belyakova1@g.nsu.ru',
    description='Modele 3 Task 2',
    license='Apache-2.0 license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'action_turtle_server = action_turtle_commands.action_turtle_server:main',
            'action_turtle_client = action_turtle_commands.action_turtle_client:main',
        ],
    },
)
