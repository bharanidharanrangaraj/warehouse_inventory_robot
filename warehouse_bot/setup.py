import os
from glob import glob

from setuptools import setup

package_name = 'warehouse_bot'


def tree(src_dir):
    """Install a folder recursively, keeping sub-folders (Gazebo models need this)."""
    entries = []
    for root, _, files in os.walk(src_dir):
        if files:
            entries.append((os.path.join('share', package_name, root),
                            [os.path.join(root, f) for f in files]))
    return entries


setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
        ('share/' + package_name + '/maps', glob('maps/*')),
        ('share/' + package_name + '/web', glob('web/*')),
    ] + tree('models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bharani Dharan Rangaraj',
    maintainer_email='you@example.com',
    description='Warehouse Inventory Robot - fetch-on-order and inventory audit, '
                'ROS 2 Humble + Gazebo + Nav2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zone_detector = warehouse_bot.zone_detector:main',
            'marker_scanner = warehouse_bot.marker_scanner:main',
            'inventory_manager = warehouse_bot.inventory_manager:main',
            'mission_manager = warehouse_bot.mission_manager:main',
            'fetch_manager = warehouse_bot.fetch_manager:main',
            'item_handler = warehouse_bot.item_handler:main',
            'web_server = warehouse_bot.web_server:main',
            'web_stdio_proxy = warehouse_bot.web_stdio_proxy:main',
            'generate_world = warehouse_bot.generate_world:main',
        ],
    },
)
