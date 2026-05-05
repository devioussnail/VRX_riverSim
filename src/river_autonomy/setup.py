from setuptools import find_packages, setup

package_name = 'river_autonomy'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/simple_autonomy.launch.py']),
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
            'simple_autonomy_node = river_autonomy.simple_autonomy_node:main',
        ],
    },
)
