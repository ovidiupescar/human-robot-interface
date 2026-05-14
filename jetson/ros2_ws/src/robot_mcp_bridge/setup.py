from setuptools import find_packages, setup

package_name = 'robot_mcp_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        # MCP + HTTP stack. Pinned-but-not-locked; bumps go via uv-lock-style
        # refresh when needed.
        'mcp>=1.2',
        'fastapi>=0.110',
        'uvicorn[standard]>=0.27',
        'websockets>=12.0',
        'pydantic>=2.0',
    ],
    zip_safe=True,
    maintainer='Ovidiu Pescar',
    maintainer_email='ovidiu@artoriuslabs.com',
    description='ROS2 ↔ Hermes Agent MCP bridge daemon',
    license='MIT',
    entry_points={
        'console_scripts': [
            'daemon = robot_mcp_bridge.daemon:main',
        ],
    },
)
