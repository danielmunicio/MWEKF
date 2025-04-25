from setuptools import find_packages, setup

package_name = 'camera_only'

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
            'sensor_fusion = camera_only.find_cone_cluster:main',
            'camera_only = camera_only.realsense_camera_only:main'
        ],
    },
)
