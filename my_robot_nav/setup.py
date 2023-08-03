from setuptools import setup

package_name = 'my_robot_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
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
            'prius_cmd_vel_mirror = my_robot_nav.prius_cmd_vel_mirror:main',
            'prius_cmd_vel_traffic_light = my_robot_nav.prius_cmd_vel_traffic_light:main',
        ],
    },
)
