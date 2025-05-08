from setuptools import find_packages, setup

package_name = 'graphslam_global'

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
    maintainer='tpark',
    maintainer_email='tpark@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'graphslam_global = graphslam_global.main:main',
            'linear_tester = graphslam_global.linear_tester:main',
        ],
    },
)
