from setuptools import find_packages, setup

package_name = 'robot_audio'

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
    description='Audio capture, playback, TTS',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'audio_capture = robot_audio.audio_capture_node:main',
            'audio_player  = robot_audio.audio_player_node:main',
            'tts_service   = robot_audio.tts_service_node:main',
        ],
    },
)
