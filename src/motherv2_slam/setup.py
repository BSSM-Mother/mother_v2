from setuptools import setup

package_name = 'motherv2_slam'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@mother.local',
    description='SLAM-based object localization for MotherV2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'slam_localization_node = motherv2_slam.slam_localization_node:main',
            'explore_node = motherv2_slam.explore_node:main',
        ],
    },
)
