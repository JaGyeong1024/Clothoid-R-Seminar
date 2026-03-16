from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    car_name_arg = DeclareLaunchArgument(
        'car_name',
        default_value='car1',
        description='시뮬레이터 차량 이름 (spawn_car.launch.py의 car_name과 동일)'
    )

    pipeline_node = Node(
        package='lidar_detection',
        executable='lidar_pipeline',
        name='lidar_pipeline',
        parameters=[{'car_name': LaunchConfiguration('car_name')}],
        output='screen',
    )

    return LaunchDescription([car_name_arg, pipeline_node])
