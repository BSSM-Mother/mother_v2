from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'motherv2_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', glob('models/*.onnx') + glob('models/*.tflite')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'detection_node = motherv2_detection.detection_node:main',
        ],
    },
)
