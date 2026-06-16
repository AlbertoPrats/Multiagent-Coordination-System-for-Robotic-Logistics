import os
import yaml
import xacro
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch_ros.actions import Node

def make_nav2_nodes(robot_ns, nav2_params_file_template, nav2_map, pos_y):
    # Read and substitute the template for this specific robot
    with open(nav2_params_file_template, 'r') as f:
        content = f.read()
 
    # Sustituir namespace (ej: 'namespace/' → 'robot_0/')
    content = content.replace('namespace/', f'{robot_ns}/')
 
    # Sustituir pos_y con el valor real (float → string)
    content = content.replace('rob_pos_y', str(pos_y))
 
    # Ruta fija y predecible — se sobreescribe en cada launch
    print(robot_ns)
    output_path = os.path.join(get_package_share_directory('proyecto'), f'config/{robot_ns}_nav2.yaml')
    with open(output_path, 'w') as f:
        f.write(content)
 
    print(f'[make_nav2_nodes] Params escritos en: {output_path}')


    tf_remaps = [('tf', '/tf'), ('tf_static', '/tf_static'), (f'/{robot_ns}/map', '/map')]

    return [
        Node(package='nav2_behaviors',   executable='behavior_server',  name='behavior_server',  namespace=robot_ns, parameters=[output_path], remappings=tf_remaps, output='log'),
        Node(package='nav2_map_server',  executable='map_server',       name='map_server',       namespace=robot_ns, parameters=[output_path, {'yaml_filename': nav2_map}], remappings=tf_remaps, output='log'),
        #Node(package='nav2_amcl',        executable='amcl',             name='amcl',             namespace=robot_ns, parameters=[tmp.name], remappings=tf_remaps, output='screen'),
        Node(package='nav2_planner',     executable='planner_server',   name='planner_server',   namespace=robot_ns, parameters=[output_path], remappings=tf_remaps, output='log'),
        Node(package='nav2_controller',  executable='controller_server', name='controller_server', namespace=robot_ns, parameters=[output_path], remappings=tf_remaps, output='log'), #arguments=['--ros-args', '--log-level', 'debug']),
        Node(package='nav2_smoother',    executable='smoother_server',  name='smoother_server',  namespace=robot_ns, parameters=[output_path], remappings=tf_remaps, output='log'),
        Node(package='nav2_bt_navigator',executable='bt_navigator',     name='bt_navigator',     namespace=robot_ns, parameters=[output_path], remappings=tf_remaps, output='log'),
        Node(package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor', namespace=robot_ns, parameters=[output_path], remappings=tf_remaps, output='log'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager', namespace=robot_ns, output='log',
             parameters=[{'use_sim_time': True, 'autostart': True,
                          'node_names': ['map_server','behavior_server','planner_server',
                                         'controller_server','smoother_server','bt_navigator','collision_monitor']}]),
    ]

def launch_moviles(n_robots, urdf_path,rviz_config_path,world_path,gazebo_config_path,nav2_params_file,nav2_map, models_path):
    
    colours=["red", "blue", "green", "dark_red"]
    lista_nodos=[]
    #Indicación para gazebo de donde buscar los modelos incluidos en el mundo
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.pathsep, models_path]
    )
    lista_nodos.append(set_gz_resource_path)
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': f'-r {world_path}',
            'on_exit_shutdown': 'true',
            'use_sim_time': 'true',
            'use_ros2_control': 'true'
        }.items()
    )
    lista_nodos.append(gazebo_launch)


    for i in range(n_robots):
        
        if(i <= 3):
            colour=colours[i]
        else:
            colour=colours[3]

        # Procesar Xacro pasando el namespace como argumento
        robot_description_content=None

        robot_description_content = Command([
            'xacro ', urdf_path, ' ',
            'namespace:=', f'robot_{i}', ' ',
            'colour_base:=', colour
        ])

        # Nodo Robot State Publisher: Publica la estructura interna del robot
        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=f'robot_{i}',
            name=f'robot_state_publisher_{i}',
            output='log',
            parameters=[
                {'robot_description': robot_description_content}, 
                {'ignore_timestamp': True},
                {'use_sim_time': True},
            ],
            remappings=[
                ('/robot_description', f'/robot_{i}/robot_description'),
                ('/joint_states', f'/robot_{i}/joint_states'),
            ]
            
        )

        # Nodo para spawnear el robot en la posición de Gazebo
        spawn_robot = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', f'/robot_{i}/robot_description',
                '-name', f'robot_{i}',
                '-x', '-6',
                '-y', f'{0.5-i}',
            ],
            output='log'
        )
        # Transformaciones para tener cada robot como hijo de u odom y luego ese odom que esté referenciado a map.
        # El odom inicia donde inica el robot, por eso se pone que la transformación es nula, pero entre el odom y 
        # map se le pone el desplazamiento.
        
        '''
        static_tf_robot_odom = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'static_tf_odom_{i}',
            arguments=['0', '0', '0', '0', '0', '0', f'robot_{i}/odom', f'robot_{i}/base_footprint_link']
        )
        '''
        '''
        ground_truth_node = Node(
            package='proyecto',                    # ← your real package name
            executable='correccionPos.py',
            arguments=[f'robot_{i}'],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )
        lista_nodos.append(ground_truth_node)
        ''' 
        
        static_tf_odom_map = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', f'robot_{i}/odom']
        )
        #lista_nodos.append(static_tf_robot_odom)
        lista_nodos.append(static_tf_odom_map)
        
        lista_nodos.append(robot_state_publisher_node)
        lista_nodos.append(spawn_robot)

        nav2_nodes=make_nav2_nodes(f'robot_{i}', nav2_params_file, nav2_map, 0.5-i)

        delayed_nav2 = TimerAction(period=3.0, actions=nav2_nodes)
        lista_nodos.append(delayed_nav2)

    GenerarConfigYaml(gazebo_config_path, n_robots)

    # 5. Gazebo Bridge (Mapeo de tópicos entre ROS2 y Gazebo)
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': gazebo_config_path, 
                    'use_sim_time':True}],
        arguments=[
            '/world/cocina_robotica_recreada/set_pose@ros_gz_interfaces/srv/SetEntityPose'
        ]
    )
    lista_nodos.append(gz_bridge)

    tf_filter_node = Node(
        package='proyecto',
        executable='tf_filter.py',
        arguments=[str(n_robots)],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    lista_nodos.append(tf_filter_node)
    
    '''
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        output='log'
    )
    lista_nodos.append(rviz2)
    '''
    
    ground_truth_node = Node(
        package='proyecto',
        executable='correccionPos.py',
        arguments=[str(n_robots)],          # número total, ej. "3"
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    lista_nodos.append(ground_truth_node)

    wander= Node(
        package='entrega_final',
        executable='wonder.py', # El nombre exacto del archivo
        name='motor_movimiento',
        output='log',
        parameters=[
            {'num_robots': n_robots}
        ] # Opcional
    )
    return lista_nodos

def GenerarConfigYaml(path, num_rob):
    config_data=[]

    aux={
        'ros_topic_name': '/clock',
        'gz_topic_name': '/clock', 
        'ros_type_name': 'rosgraph_msgs/msg/Clock', 
        'gz_type_name': 'gz.msgs.Clock', 
        'direction': 'GZ_TO_ROS'
    }

    config_data.append(aux)

    for i in range(num_rob):
        aux={
            'ros_topic_name': f'/robot_{i}/odom',
            'gz_topic_name': f'/model/robot_{i}/odometry', 
            'ros_type_name': 'nav_msgs/msg/Odometry', 
            'gz_type_name': 'gz.msgs.Odometry', 
            'direction': 'GZ_TO_ROS'
        }

        config_data.append(aux)
        
        
        aux = {
            'ros_topic_name': f'/robot_{i}/ground_truth',
            'gz_topic_name': f'/model/robot_{i}/pose',
            'ros_type_name': 'tf2_msgs/msg/TFMessage',
            'gz_type_name': 'gz.msgs.Pose_V',
            'direction': 'GZ_TO_ROS'
        }
        
        config_data.append(aux)
        
        
        aux={
            'ros_topic_name': f'/tf_unfiltered',
            'gz_topic_name': f'/model/robot_{i}/tf', 
            'ros_type_name': 'tf2_msgs/msg/TFMessage', 
            'gz_type_name': 'gz.msgs.Pose_V', 
            'direction': 'GZ_TO_ROS'
        }
        
        config_data.append(aux)
        
        aux={
            'ros_topic_name': f'/robot_{i}/cmd_vel',
            'gz_topic_name': f'/model/robot_{i}/cmd_vel', 
            'ros_type_name': 'geometry_msgs/msg/Twist', 
            'gz_type_name': 'gz.msgs.Twist', 
            'direction': 'ROS_TO_GZ'
        }

        config_data.append(aux)

        aux={
            'ros_topic_name': f'/robot_{i}/cameraImagen/image_raw',
            'gz_topic_name': f'/robot_{i}/camara', 
            'ros_type_name': 'sensor_msgs/msg/Image', 
            'gz_type_name': 'gz.msgs.Image', 
            'direction': 'GZ_TO_ROS'
        }

        config_data.append(aux)

        aux={
            'ros_topic_name': f'/robot_{i}/cameraImagen/camera_info',
            'gz_topic_name': f'/robot_{i}/camera_info', 
            'ros_type_name': 'sensor_msgs/msg/CameraInfo', 
            'gz_type_name': 'gz.msgs.CameraInfo', 
            'direction': 'GZ_TO_ROS'
        }

        config_data.append(aux)

        aux={
            'ros_topic_name': f'/robot_{i}/lidar/puntos',
            'gz_topic_name': f'/robot_{i}/lidar/points', 
            'ros_type_name': 'sensor_msgs/msg/PointCloud2', 
            'gz_type_name': 'gz.msgs.PointCloudPacked', 
            'direction': 'GZ_TO_ROS'
        }

        config_data.append(aux)

        aux={
            'ros_topic_name': f'/robot_{i}/laser',
            'gz_topic_name': f'/robot_{i}/lidar', 
            'ros_type_name': 'sensor_msgs/msg/LaserScan', 
            'gz_type_name': 'gz.msgs.LaserScan', 
            'direction': 'GZ_TO_ROS'
        }

        config_data.append(aux)
        
        aux = {
            'ros_topic_name': f'/robot_{i}/joint_states',
            'gz_topic_name': f'/world/robot_world/model/robot_{i}/joint_state', 
            'ros_type_name': 'sensor_msgs/msg/JointState', 
            'gz_type_name': 'gz.msgs.Model',
            'direction': 'GZ_TO_ROS'
        }

        config_data.append(aux)
        
    if os.path.exists(path):
        print(path)
    else:
        print(path)
    with open(path, 'w') as file:
        yaml.safe_dump(config_data, file)

def launch_ABB(xacro_file):
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc, mappings={})
    robot_description_config = doc.toxml()
    robot_description = {'robot_description': robot_description_config}

    # ROBOT STATE PUBLISHER NODE:
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
            {'ignore_timestamp': True},
            {"use_sim_time": True}
        ]
    )

    # SPAWN ROBOT TO GAZEBO:
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=[
                                    '-topic', 'robot_description',
                                    '-entity', 'irb120',
                                    '-x', '-0.34',
                                    '-y', '0.0',
                                    '-z', '0.7',
                                    '-Y', '-1.5708',
                                ],
                        output='screen')

    # ***** CONTROLLERS ***** #
    # Joint STATE BROADCASTER:
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    # Joint TRAJECTORY Controller:
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["irb120_controller", "-c", "/controller_manager"],
    )

    # === SCHUNK EGP-64 === #
    egp64left_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["egp64_finger_left_controller", "-c", "/controller_manager"],
    )
    egp64right_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["egp64_finger_right_controller", "-c", "/controller_manager"],
    )

    return [
        node_robot_state_publisher,
        spawn_entity,

        RegisterEventHandler(
            OnProcessExit(
                target_action = spawn_entity,
                on_exit = [
                    joint_state_broadcaster_spawner,
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action = joint_state_broadcaster_spawner,
                on_exit = [
                    joint_trajectory_controller_spawner,
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action = joint_trajectory_controller_spawner,
                on_exit = [
                    egp64left_controller_spawner,
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action = egp64left_controller_spawner,
                on_exit = [
                    egp64right_controller_spawner,
                ]
            )
        ),

    ]

def generate_launch_description():

    n_robots=input("Introduzca el número de robots a generar:")

    try:
        n_robots = int(n_robots)
    except ValueError:
        print("El valor introducido no parece ser un número válido. Usando valor por defecto (2).")
        n_robots=2

    if n_robots < 0:
        n_robots = -n_robots
        print(f"El valor introducido es negativo. Se generaran {n_robots} robots.")

    pkg_share = get_package_share_directory('proyecto')
    
    #Direcciones relativas de los ficheros a lanzar
    urdf_path = os.path.join(pkg_share, 'urdf/robot_movil', 'robot.xacro')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'rviz_config.rviz')
    world_path = os.path.join(pkg_share, 'world', 'mapa_1ABB.sdf')
    gazebo_config_path = os.path.join(pkg_share, 'config', 'config.yaml')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2.yaml')
    nav2_map = os.path.join(pkg_share, 'world', 'mapa_cocina.yaml')
    models_path = os.path.join(pkg_share, 'models')
    
    
    
    lista_nodos = launch_moviles(n_robots, urdf_path,rviz_config_path,world_path,gazebo_config_path,nav2_params_file,nav2_map, models_path)
    
    xacro_file = os.path.join(pkg_share,
                              'urdf/IRB120_ABB',
                              'irb120.urdf.xacro')
    
    lista_nodos.extend(launch_ABB(xacro_file))
    
    return LaunchDescription(lista_nodos)



