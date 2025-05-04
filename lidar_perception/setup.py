from setuptools import find_packages, setup

package_name = 'lidar_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daniel',
    maintainer_email='danielmunicio360@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_perception = lidar_perception.main:main',
            'save_points = lidar_perception.save_points_to_pc:main',
            'sensor_fusion = lidar_perception.find_cone_cluster:main',
            'find_ground = lidar_perception.find_ground_gui:main',
            'save_pointcloud = lidar_perception.save_pointcloud:main'
        ],
    },
)
