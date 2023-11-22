	# Copyright 2018 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os
from launch import LaunchDescription
from launch.actions import Shutdown,IncludeLaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, EmitEvent
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.events import matches_action
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory




config = os.path.join(
        get_package_share_directory('motor_ros_driver'),
        'config',
        'params.yaml'
        )
motor_ns = 'motor_1_node'
def generate_launch_description():
    ld = LaunchDescription()
    motor_1_node = Node(package='motor_ros_driver', executable='talker_node',
                      name='talker_launch', namespace='', output='screen', parameters=[{'my_int': 5}])    
   
                
    ld.add_action(motor_1_node)

    return ld
