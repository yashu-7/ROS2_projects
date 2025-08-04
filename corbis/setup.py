import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'corbis'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.xacro'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/**/*', recursive=True)),
        (os.path.join('share', package_name, 'world'), glob(os.path.join('world', '*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),

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
            'gui = corbis.gui_v2:main',
            'save_coords = corbis.save_coordinates:main',
            'save_map=corbis.save_map:main',
            'navigation=corbis.navigate:main'
        ],
    },
)
