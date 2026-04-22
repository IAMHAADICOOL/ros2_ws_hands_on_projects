from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'cerveza_tb_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.*")),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jay',
    maintainer_email='vasutorn.si@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'control_tb = cerveza_tb_planning.control_tb:main',
            'grid_mapping = cerveza_tb_planning.grid_mapping:main',
            'rrt_motion_planning = cerveza_tb_planning.rrt_motion_planning:main',
        ],
    },
)
