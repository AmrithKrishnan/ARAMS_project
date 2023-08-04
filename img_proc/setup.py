from setuptools import setup
import os
from glob import glob

package_name = 'img_proc'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amrith',
    maintainer_email='amrith.krishnan@alumni.fh-aachen.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crop_raw_img = img_proc.crop_raw_img:main',
            'traffic_light_detect = img_proc.traffic_light_detect:main',
        ],
    },
)
