from setuptools import setup
import os
from glob import glob
from setuptools import find_packages

package_name = 'video_capture'

setup(
 name=package_name,
 version='0.0.0',
 packages=[package_name],
 data_files=[
     ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
     ('share/' + package_name, ['package.xml']),
     (os.path.join('share', package_name), glob('./launch/*.launch.py')),
     #(os.path.join('share', package_name), glob('./config/*config.rviz'))  
   ],
 install_requires=['setuptools'],
 zip_safe=True,
 maintainer='TODO',
 maintainer_email='TODO',
 description='TODO: Package description',
 license='TODO: License declaration',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
             'resize_node = video_capture.resize_node:main',
             'capture_node = video_capture.capture_node:main',
             'process_node = video_capture.process_node:main'
     ],
   },
)
