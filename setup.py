from setuptools import setup
from glob import glob

package_name = 'Robomaster_Maze'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
    	#('/home/usi/dev_ws/build/Robomaster_Maze/build/lib/Robomaster_Maze')
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        ('share/' + package_name + '/launch', glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='usi',
    maintainer_email='eneregemen@gmail.com',
    description='ROS package that implements the RoboMaster-Maze-Mapping',
    license='MIT',

    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                    'controller_node = Robomaster_Maze.controller_node:main'
                    
        ],
    },
)
