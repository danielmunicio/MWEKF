from setuptools import find_packages, setup

package_name = 'low_level'

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
    maintainer='reid',
    maintainer_email='reid@xz.ax',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sbw_can = low_level.sbw_can:main',
            'bbw_ser = low_level.bbw_ser:main',
        ],
    },
)
