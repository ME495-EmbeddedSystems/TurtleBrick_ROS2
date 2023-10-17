from setuptools import find_packages, setup

package_name = 'turtle_brick'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/show_turtle.launch.xml']),
        ('share/' + package_name, ['urdf/slidebot.urdf']),
        ('share/' + package_name, ['config/turtle_test.rviz']),
        ('share/' + package_name, ['launch/show_turtle_launch.py']),
        ('share/' + package_name, ['urdf/robot.urdf.xacro']),
        ('share/' + package_name, ['launch/run_turtle.launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jialuyu',
    maintainer_email='jialuyu2024@u.northwestern.edu',
    description='A turtle brick package allow turtle to catch bricks',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_robot = turtle_brick.turtle_node: main'
        ],
    },
)
