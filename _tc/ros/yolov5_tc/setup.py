import os
from glob import glob
from setuptools import setup

package_name = 'yolov5_tc'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[('share/ament_index/resource_index/packages',
                 ['resource/' + package_name]),
                (os.path.join('share', package_name), ['package.xml']),
                (os.path.join('share', package_name,
                              'launch'), glob('launch/*launch.[pxy][yma]*')),
                (os.path.join('share', package_name,
                              'config'), glob('config/*.yaml'))],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Raphael van Kempen',
    maintainer_email='vankempen@thinking-cars.de',
    description='Thinking Cars YOLOv5 ROS2 package',
    license='CC-BY-NC-4.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts':
        ['inference_node = yolov5_tc.inference_node:main',
         'image_publisher = yolov5_tc.image_publisher:main'],
    },
)
