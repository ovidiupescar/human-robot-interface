from setuptools import find_packages, setup

package_name = 'robot_reflex'

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
    description='Fast reactive behaviors',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reflex = robot_reflex.reflex_node:main',
            'intent_shortcut = robot_reflex.intent_shortcut_node:main',
        ],
    },
)
