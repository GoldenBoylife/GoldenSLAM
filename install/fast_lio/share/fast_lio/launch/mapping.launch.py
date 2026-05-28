# ros2 launch에서 쓰는 것들
import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node



## 어떤 노드를 어떤 옵션으로 실행할지 설명서 만들어서 돌려주는 함수
def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    # get_package_share_directory() : ros2패키지의 share 디렉토리 경로를 찾는 함수


    ## config 기본 경로들 
    default_config_path = os.path.join(package_path, 'config')
    # 현재 패키지 폴더 안에 있는 config라는 이름의 디렉토리 경로 만들기
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')
        # 현 패키지의 rviz 폴더 안에 있는 fastlio.rviz 설정 파일 가리킴.

    ## 사용자로부터 입력받을 매개변수를 정의하고 저장하는 바구니
    use_sim_time = LaunchConfiguration('use_sim_time')  # 시뮬레이션 시간 사용 여부
    config_path = LaunchConfiguration('config_path')    # 설정 폴더 경로
    config_file = LaunchConfiguration('config_file')    # 설정 파일 이름 
    rviz_use = LaunchConfiguration('rviz')              # Rviz 실행 여부 
    rviz_cfg = LaunchConfiguration('rviz_cfg')          # Rviz 설정 파일 경로 
    

    ## DeclareLaunchArgument : 런치할때 밖에서도 값을 넘길 수 잇게 만드는 옵션 선언
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        #이 변수에 대한  기본값 false
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='mid360.yaml',
        ## 이launch파일은 mid360용이구나. 
        description='Config file'
    )
    # rviz띄울지 말지 결정
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )


    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )
    
    ## Node1 : fast_lio 정의
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        # 실행할 바이너리 이름
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {'use_sim_time': use_sim_time}],
                    # 파라미터 넘기기
                    # config 폴더 경로와 파일 이름 합쳐서 파라미터로 보냄. 
                    # use_sim_time : 표준 에약어,
                    # yaml 파일 통째로 노드에 던져준다. 
                    
        output='screen'
    )

    ## Node2 : Rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
        # IfCondition : 조건부 실행, Rviz켤지 안켤지 결정.
    )

    ld = LaunchDescription()
    # launch파일의 최종 결과물, 여기에 실행할 액션들을 차곡차곡 넣어서 반환함
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)

    ld.add_action(fast_lio_node)
    ld.add_action(rviz_node)

    return ld
    # 최종반환, 이걸 ros2 시스템이 읽고 실행함. 
