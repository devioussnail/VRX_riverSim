from setuptools import find_packages, setup

package_name = 'river_autonomy'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/perception.launch.py']),
        ('share/' + package_name + '/launch', ['launch/planning.launch.py']),
        ('share/' + package_name + '/launch', ['launch/control.launch.py']),
        ('share/' + package_name + '/launch', ['launch/autonomy_stack.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Noah Mason',
    maintainer_email='no-reply@example.com',
    description='Simple ROS 2 autonomy package for VRX river experiments.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_node = river_autonomy.perception_node:main',
            'planning_node = river_autonomy.planning_node:main',
            'control_node = river_autonomy.control_node:main',
        ],
    },
)
