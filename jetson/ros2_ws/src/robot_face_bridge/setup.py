from setuptools import find_packages, setup

package_name = 'robot_face_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ovidiu Pescar',
    maintainer_email='ovidiu@artoriuslabs.com',
    description='Serial JSON bridge between ROS2 and ESP32 face board',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'face_bridge = robot_face_bridge.face_bridge_node:main',
        ],
    },
)
