from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mujoco_bootcamp'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *[
            (os.path.join('share', package_name, os.path.dirname(f)), [f])
            for f in glob('mjcf/**/*', recursive=True)
            if os.path.isfile(f)
        ],
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jwj',
    maintainer_email='m6488j@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_mujoco_2d_manipulator = mujoco_bootcamp.run_mujoco_2d_manipulator:main',
            'run_mujoco_diff_drive = mujoco_bootcamp.run_mujoco_diff_drive:main'
        ],
    },
)
