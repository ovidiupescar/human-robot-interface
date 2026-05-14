from setuptools import find_packages, setup

package_name = 'robot_graph'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'kuzu', 'numpy'],
    zip_safe=True,
    maintainer='Ovidiu Pescar',
    maintainer_email='ovidiu@artoriuslabs.com',
    description='Knowledge graph (KuzuDB) + identity/memory/location services + scene recognizer',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'graph_service     = robot_graph.graph_service_node:main',
            'voice_identifier  = robot_graph.voice_identifier_node:main',
            'scene_recognizer  = robot_graph.scene_recognizer_node:main',
            'identity_fusion   = robot_graph.identity_fusion_node:main',
            'context_prefetch  = robot_graph.context_prefetch_node:main',
        ],
    },
)
