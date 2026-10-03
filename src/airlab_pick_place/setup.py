from glob import glob

from setuptools import find_packages, setup

package_name = 'airlab_pick_place'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',
    maintainer_email='developer@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pregrasp = airlab_pick_place.pregrasp:main',
            'pick_place_server = airlab_pick_place.pick_place_server:main',
            'collect_episodes = airlab_pick_place.collect_episodes:main',
            'pi05_client = airlab_pick_place.pi05_client:main',
        ],
    },
)
