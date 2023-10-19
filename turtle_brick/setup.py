from setuptools import find_packages, setup

package_name = 'turtle_brick'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 
                                   'launch/show_turtle.launch.xml', 
                                   'urdf/slidebot.urdf',
                                   'config/turtle_test.rviz',
                                   'launch/show_turtle_launch.py',
                                   'urdf/robot.urdf.xacro',
                                   'launch/run_turtle.launch.xml',
                                   'launch/turtle_arena.launch.xml',
                                   ]),
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
         
            'turtle_robot = turtle_brick.turtle_node: main',
            'Arena = turtle_brick.Arena:main',
            'Catcher = turtle_brick.Control:main',
        ],
    },
)
