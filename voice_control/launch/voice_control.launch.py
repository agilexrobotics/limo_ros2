from doctest import OutputChecker
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node
import os


def generate_launch_description():
    plugin_path = os.path.join(get_package_share_path("voice_control"),"Plugins")

    declare_plugin_path = DeclareLaunchArgument(name="plugin_path",default_value=plugin_path)
    declare_mic_index = DeclareLaunchArgument(name="mic_index",default_value=TextSubstitution(text='11'))
    declare_speaker_index = DeclareLaunchArgument(name="speakcer_index",default_value=TextSubstitution(text='11'))
    declare_hot_word = DeclareLaunchArgument(name="hot_word", default_value="")
    declare_authentic_key = DeclareLaunchArgument(name="authentic_key", default_value="")
    declare_recognition_engine = DeclareLaunchArgument(name="recognition_engine", default_value="pocketsphinx")

    plugin_path=LaunchConfiguration("plugin_path",default=declare_plugin_path)
    mic_index = LaunchConfiguration("mic_index", default=declare_mic_index)
    speaker_index = LaunchConfiguration("speaker_index", default=declare_speaker_index)
    hot_word = LaunchConfiguration("hot_word", default=declare_hot_word)
    authentic_key = LaunchConfiguration("authentic_key", default=declare_authentic_key)
    recognition_engine = LaunchConfiguration("recognition_engine", default=declare_recognition_engine)

    voice_control = Node(
        package="voice_control",
        executable="voice_control",
        output='screen',
        arguments=['--ros-args', '--log-level','info'],
        parameters=[
            {
                "mic_index":mic_index,
                "speaker_index":speaker_index,
                "hot_word":hot_word,
                "authentic_key":authentic_key,
                "recognition_engine":recognition_engine,
                "plugin_path":plugin_path
            }
        ]
    )
    return LaunchDescription([
        declare_plugin_path,
        declare_mic_index,
        declare_speaker_index,
        declare_hot_word,
        declare_authentic_key,
        declare_recognition_engine,
        voice_control
    ])