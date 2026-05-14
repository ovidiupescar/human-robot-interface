from setuptools import find_packages, setup

package_name = 'robot_perception'

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
    description='VAD, STT, vision',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_activity      = robot_perception.voice_activity_node:main',
            'speech_recognizer   = robot_perception.speech_recognizer_node:main',
            'wake_word           = robot_perception.wake_word_node:main',
            'addressee_estimator = robot_perception.addressee_estimator_node:main',
            'perception_executor = robot_perception.perception_executor:main',
            'language_resolver  = robot_perception.language_resolver_node:main',
            'vision_capture     = robot_perception.vision_capture_node:main',
        ],
    },
)
