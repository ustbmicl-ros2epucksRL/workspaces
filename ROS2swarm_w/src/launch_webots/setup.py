import os
from glob import glob
from setuptools import setup

package_name = 'launch_webots'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Webots 资产：URDF、Proto、纹理、网格（源码在 assets/ 目录）
        (os.path.join('share', package_name, 'assets'), glob('assets/*.urdf')),
        (os.path.join('share', package_name, 'assets'), glob('assets/*.proto')),
        (os.path.join('share', package_name, 'assets', 'textures'), glob('assets/textures/*.jpg') + glob('assets/textures/*.png')),
        (os.path.join('share', package_name, 'assets', 'RpLidarA2'), glob('assets/RpLidarA2/*.proto')),
        (os.path.join('share', package_name, 'assets', 'RpLidarA2', 'meshes'), glob('assets/RpLidarA2/meshes/*.obj')),
        (os.path.join('share', package_name, 'assets', 'RpLidarA2', 'icons'), glob('assets/RpLidarA2/icons/*.png')),
        # ROS2 控制器配置（仍在 resource/ 目录）
        (os.path.join('share', package_name, 'resource'), glob('resource/*.yml')),
        # 世界文件与地图
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.wbt')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Marian Begemann',
    author_email='marian.begemann(at)student.uni-luebeck.de',
    maintainer='Tanja Kaiser',
    maintainer_email='kaiser(at)iti.uni-luebeck.de',
    license='apache-2.0',
    description='Launch script for webots simulation and the ros2swarm',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                    'add_bot_node = launch_webots.add_bot_node:main',
                    'epuck_node = launch_webots.epuck_node:main',
                    'global_info_supervisor = launch_webots.global_info_supervisor:main',
                    'robosar_bridge_adapter = launch_webots.robosar_bridge_adapter:main',
        ],
    },
)
