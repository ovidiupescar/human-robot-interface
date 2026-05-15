from setuptools import setup

package_name = 'robot_realtime'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ovidiu Pescar',
    maintainer_email='ovidiu@example.com',
    description='Gemini Live realtime audio bridge for the ROS2 graph.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'realtime_bridge = robot_realtime.realtime_bridge_node:main',
        ],
    },
)
