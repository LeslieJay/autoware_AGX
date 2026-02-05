from setuptools import setup

package_name = 'service_set_goal'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/goal_points.yaml']),
        ('share/' + package_name + '/launch', ['launch/set_goal.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='huanyue',
    maintainer_email='1531442012@qq.com',
    description='A ROS2 node to set route points via service call',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'set_goal_node = service_set_goal.set_goal_node:main',
        ],
    },
)

