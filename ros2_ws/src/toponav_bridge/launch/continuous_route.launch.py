"""Three actual ROS processes; simulation scene is ONLY passed to world owner."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,RegisterEventHandler,EmitEvent
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    public=LaunchConfiguration('public');output=LaunchConfiguration('output')
    world=Node(package='toponav_bridge',executable='bridge_node',namespace='toponav',name='world',
               arguments=['world','--public',public,'--output',output,'--scenario',LaunchConfiguration('scenario')],output='screen')
    nav=Node(package='toponav_bridge',executable='bridge_node',namespace='toponav',name='navigation',
             arguments=['navigation','--public',public,'--output',output],output='screen',parameters=[{'use_sim_time':True}])
    feedback=Node(package='toponav_bridge',executable='bridge_node',namespace='toponav',name='feedback',
                  arguments=['feedback','--public',public,'--output',output],output='screen',
                  parameters=[{'drop_feedback':ParameterValue(LaunchConfiguration('drop_feedback'),value_type=bool)}])
    fault=Node(package='toponav_bridge',executable='bridge_node',namespace='toponav',name='fault_injector',
               arguments=['fault','--public',public,'--output',output,'--fault-mode',LaunchConfiguration('fault_mode'),'--fault-tick',LaunchConfiguration('fault_tick')],
               condition=IfCondition(LaunchConfiguration('inject_command')),output='screen')
    return LaunchDescription([DeclareLaunchArgument('public'),DeclareLaunchArgument('output'),DeclareLaunchArgument('scenario'),
                              DeclareLaunchArgument('drop_feedback',default_value='false'),DeclareLaunchArgument('inject_command',default_value='false'),
                              DeclareLaunchArgument('fault_mode',default_value='replay_command'),DeclareLaunchArgument('fault_tick',default_value='180'),
                              world,nav,feedback,fault,RegisterEventHandler(OnProcessExit(target_action=world,on_exit=[EmitEvent(event=Shutdown(reason='world finished'))]))])
