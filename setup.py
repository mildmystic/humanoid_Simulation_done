import os
from glob import glob
from setuptools import setup

package_name = 'humannew'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # Now points to the humannew/ dir
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        
        # Add these lines if i create config or urdf dirs later
        
        #(os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ashish',
    maintainer_email='mittalashu878@gmail.com',
    description='NAO humanoid simulation package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nao_wave_node = humannew.nao_wave_node:main',  
            'localization_node = humannew.localization_node:main',
        ],
    },
)