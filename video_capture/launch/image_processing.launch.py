import os
import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration



def generate_launch_description():

    # args that can be set from the command line or a default will be used
    # launch_arg = DeclareLaunchArgument(
    #         'image_topic',
    #         default_value='/camera/image_raw',
    #         description='Input image topic'
    #     ),
    

    process_node = launch_ros.actions.Node(
        package="video_capture",
        executable="process_node",   
    )
    capture_node = launch_ros.actions.Node(
        package="video_capture",
        executable="capture_node",   
    )

    resize_node = launch_ros.actions.Node(
        package="video_capture", 
        executable="resize_node",
    )

    base_path = os.path.realpath(get_package_share_directory('video_capture')) # also tried without realpath
    rviz_path=base_path+ '/config/ros2_opencv_config.rviz'

    pkg_name = 'video_capture'
    # pkg_dir = os.popen('/bin/bash -c "source /usr/share/colcon_cd/function/colcon_cd.sh && \
    # colcon_cd %s && pwd"' % pkg_name).read().strip()

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(rviz_path)]
    )



    return launch.LaunchDescription([
        #launch_arg,
        process_node,
        resize_node,
        capture_node,
        rviz_node,
    ])