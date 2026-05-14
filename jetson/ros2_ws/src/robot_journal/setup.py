from setuptools import find_packages, setup

package_name = 'robot_journal'

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
    description='Append-only journal node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'journal = robot_journal.journal_node:main',
        ],
    },
)
