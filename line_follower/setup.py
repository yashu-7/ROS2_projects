import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'line_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[xyp][may]'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.xacro'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'mesh'), glob(os.path.join('mesh', '*.stl'))),
        (os.path.join('share', package_name, 'world'), glob(os.path.join('world', '*.sdf'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yaswanth',
    maintainer_email='yaswanth@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_segment = line_follower.segment_line:main',
            'control = line_follower.control:main',
            'p_control = line_follower.p_control:main',
        ],
    },
)
