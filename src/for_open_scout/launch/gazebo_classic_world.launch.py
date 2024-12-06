import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

 
def generate_launch_description():
 
  # Constants for paths to different files and folders
  world_file_name = 'simple.world'
  
  ############ You do not need to change anything below this line #############
 
  # Set the path to different files and folders.  
  pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   

  world_path = os.path.join(
    get_package_share_directory('for_open_scout'), 
    'worlds',
    world_file_name
  )

  gazebo_models_path = os.path.join(
    get_package_share_directory('for_open_scout'), 
    'models'
  )

  env_gazebo_model_path = os.environ.get("GAZEBO_MODEL_PATH")
  if env_gazebo_model_path is None:
    os.environ["GAZEBO_MODEL_PATH"] = "/usr/share/gazebo-11/models" + ":" + gazebo_models_path
  elif ("/usr/share/gazebo-11/models" in env_gazebo_model_path) and not (gazebo_models_path in env_gazebo_model_path):
    os.environ["GAZEBO_MODEL_PATH"] = os.environ["GAZEBO_MODEL_PATH"] + ":" + gazebo_models_path
  elif not ("/usr/share/gazebo-11/models" in env_gazebo_model_path) and (gazebo_models_path in env_gazebo_model_path):
    os.environ["GAZEBO_MODEL_PATH"] = os.environ["GAZEBO_MODEL_PATH"] + ":" + "/usr/share/gazebo-11/models"


  # Launch configuration variables specific to simulation
  world = LaunchConfiguration('world')
 
  # Declare the launch arguments
  declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Whether to execute gzclient')
  
  declare_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value=world_path,
    description='Full path to the world model file to load')
 
  # Start Gazebo server
  start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    launch_arguments={'world': world}.items())
 
  # Start Gazebo client    
  start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')))
 
  # Create the launch description and populate
  ld = LaunchDescription()
 
  # Declare the launch options
  ld.add_action(declare_simulator_cmd)
  ld.add_action(declare_world_cmd)
 
  # Add any actions
  ld.add_action(start_gazebo_server_cmd)
  ld.add_action(start_gazebo_client_cmd)
 
  return ld
