from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'turtlebot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name),glob('launch/*launch.py'))
   ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rabih',
    maintainer_email='rabih.h.kiwan@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_driver_node = turtlebot_navigation.robot_driver:main',
            'wall_finder_service_node = turtlebot_navigation.wall_finder:main',
            'lap_time_action_server_node = turtlebot_navigation.lap_time_action_server:main',
            'lap_time_action_client_node = turtlebot_navigation.lap_time_action_client:main',
            'visualize_node = turtlebot_navigation.visualize:main',            
     ],
    },
)
