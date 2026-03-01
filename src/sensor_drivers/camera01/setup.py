from setuptools import find_packages, setup
import os  # ← これを冒頭に追加
from glob import glob # ← これも追加

package_name = 'camera01'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kameusagi',
    maintainer_email='kameusagi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            f'camera_sensor_node = {package_name}.camera_sensor_node:main',
        ],
    },
)
