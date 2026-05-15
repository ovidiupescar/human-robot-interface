"""Bring up the full robot stack.

Usage:
    ros2 launch robot_bringup robot.launch.py
    ros2 launch robot_bringup robot.launch.py default_language:=en
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration('face_serial_port')
    audio_in    = LaunchConfiguration('audio_input_device')
    audio_out   = LaunchConfiguration('audio_output_device')
    grouped     = LaunchConfiguration('grouped_perception')
    default_lang = LaunchConfiguration('default_language')

    return LaunchDescription([
        DeclareLaunchArgument('face_serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('audio_input_device', default_value=''),
        DeclareLaunchArgument('audio_output_device', default_value=''),
        DeclareLaunchArgument('grouped_perception', default_value='true',
            description='Run perception nodes in single process executor'),
        DeclareLaunchArgument('default_language', default_value='en',
            description='Default spoken language (en for now; ro deferred)'),

        # --- actuation / face ---
        Node(package='robot_face_bridge', executable='face_bridge', name='face_bridge',
             parameters=[{'serial_port': serial_port, 'baud_rate': 115200}],
             output='screen'),

        # --- audio I/O ---
        Node(package='robot_audio', executable='audio_capture', name='audio_capture',
             parameters=[{'device': audio_in, 'sample_rate': 16000, 'chunk_ms': 20}],
             output='screen'),
        Node(package='robot_audio', executable='audio_player', name='audio_player',
             parameters=[{'device': audio_out, 'sample_rate': 22050}],
             output='screen'),
        Node(package='robot_audio', executable='tts_service', name='tts_service',
             parameters=[{'sample_rate': 22050, 'drive_face': True,
                          'default_language': default_lang}],
             output='screen'),

        # --- perception (grouped: single process, lower IPC overhead) ---
        Node(package='robot_perception', executable='perception_executor',
             name='perception_executor', output='screen',
             parameters=[{'default_language': default_lang,
                          'allowed_languages': ['en']}],
             condition=IfCondition(grouped)),

        # --- perception (separate processes: easier debugging) ---
        Node(package='robot_perception', executable='voice_activity',
             name='voice_activity', output='screen',
             condition=UnlessCondition(grouped)),
        Node(package='robot_perception', executable='speech_recognizer',
             name='speech_recognizer',
             parameters=[{'model_size': 'small.en', 'device': 'cuda',
                          'compute_type': 'int8_float16',
                          'allowed_languages': ['en'],
                          'default_language': default_lang}],
             output='screen', condition=UnlessCondition(grouped)),
        Node(package='robot_perception', executable='wake_word',
             name='wake_word', output='screen',
             condition=UnlessCondition(grouped)),
        Node(package='robot_perception', executable='addressee_estimator',
             name='addressee_estimator', output='screen',
             condition=UnlessCondition(grouped)),
        Node(package='robot_perception', executable='language_resolver',
             name='language_resolver',
             parameters=[{'default_language': default_lang,
                          'allowed_languages': ['en']}],
             output='screen', condition=UnlessCondition(grouped)),
        Node(package='robot_perception', executable='vision_capture',
             name='vision_capture', output='screen',
             condition=UnlessCondition(grouped)),

        # --- knowledge graph ---
        Node(package='robot_graph', executable='graph_service',
             name='graph_service', output='screen'),
        Node(package='robot_graph', executable='voice_identifier',
             name='voice_identifier', output='screen'),
        Node(package='robot_graph', executable='scene_recognizer',
             name='scene_recognizer', output='screen'),
        Node(package='robot_graph', executable='identity_fusion',
             name='identity_fusion', output='screen'),
        Node(package='robot_graph', executable='context_prefetch',
             name='context_prefetch', output='screen'),

        # --- journal ---
        Node(package='robot_journal', executable='journal',
             name='journal', output='screen'),

        # --- reflex / intent ---
        Node(package='robot_reflex', executable='reflex',
             name='reflex', output='screen'),
        Node(package='robot_reflex', executable='intent_shortcut',
             name='intent_shortcut',
             parameters=[{'min_confidence': 0.85,
                          'emit_on_partial': True}],
             output='screen'),
    ])
