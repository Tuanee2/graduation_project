from setuptools import find_packages, setup

package_name = 'my_robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='parallels',
    maintainer_email='buiquoctuan18102002@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node = my_robot_controller.my_first_node:main",
            "draw_circle = my_robot_controller.draw_circle:main",
            "pose_subcriber = my_robot_controller.pose_subcriber:main",
            "turtle_controller = my_robot_controller.turtle_controller:main",
            "read_lidar = my_robot_controller.read_lidar:main",
            "controller = my_robot_controller.controller:main",
            "test_GUI = my_robot_controller.test_GUI:main",
            "gui = my_robot_controller.gui:main",
            "debug = my_robot_controller.debug:main"
        ],
    },
)
