from setuptools import setup

package_name = 'service_initialize_pose'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/initial_pose.yaml']),
        ('share/' + package_name + '/launch', ['launch/initialize_pose.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='huanyue',
    maintainer_email='1531442012@qq.com',
    description='A ROS2 node to initialize pose via service call',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'initialize_pose_node = service_initialize_pose.initialize_pose_node:main',
        ],
    },
)

