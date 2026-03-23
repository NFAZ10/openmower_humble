#!/usr/bin/env python3

from pathlib import Path
import sys
import textwrap


EXPECTED_MARKERS = {
    "custom_deps.yaml": "version: jazzy-devel",
    "package.xml": "<depend>behaviortree_cpp</depend>",
    "CMakeLists.txt": "find_package(behaviortree_cpp REQUIRED)",
    "description/robot.urdf.xacro": '<xacro:arg name="sim_mode" default="false"/>',
    "launch/gps.launch.py": "parameters=[os.path.join(get_package_share_directory(\"open_mower_next\"), 'config', 'gps.yaml')]",
    "launch/localization.launch.py": "package_path = get_package_share_directory(\"open_mower_next\")",
    "launch/nav2.launch.py": "params_file = os.path.join(get_package_share_directory(\"open_mower_next\"), 'config', 'nav2_params.yaml')",
    "launch/micro_ros_agent.launch.py": "default_device = '/dev/ttyAMA0'",
    "launch/openmower.launch.py": "robot_description_config = xacro.process_file(xacro_file, mappings={",
    "cmake/docking_helper.cmake": "nav2_msgs",
    "src/docking_helper/docking_helper_node.hpp": "#include <nav2_msgs/action/dock_robot.hpp>",
    "src/docking_helper/docking_helper_node.cpp": "dock_client_ = rclcpp_action::create_client<nav2_msgs::action::DockRobot>(this, \"/dock_robot\");",
    "src/sim/sim_node.cpp": "this->create_timer(",
    "src/docking_helper/charger_presence_charging_dock.hpp": "bool getRefinedPose(geometry_msgs::msg::PoseStamped& pose, std::string id) override;",
    "src/docking_helper/charger_presence_charging_dock.cpp": "bool ChargerPresenceChargingDock::getRefinedPose(geometry_msgs::msg::PoseStamped& pose, std::string id)",
}


REWRITES = {
    "custom_deps.yaml": """
    repositories:
      vesc:
        type: git
        url: https://github.com/sbgisen/vesc.git
        version: humble-devel
      opennav_docking:
        type: git
        url: https://github.com/open-navigation/opennav_docking.git
        version: humble
      ublox_f9p:
        type: git
        url: https://github.com/jkaflik/ublox_f9p
        version: main
      ntrip_client:
        type: git
        url: https://github.com/LORD-MicroStrain/ntrip_client
        version: ros2
    """,
    "package.xml": """
    <?xml version="1.0"?>
    <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
    <package format="3">
      <name>open_mower_next</name>
      <version>0.0.0</version>
      <description>An alternative software for Open Mower based on ROS2.</description>
      <maintainer email="jakub@kaflik.pl">Jakub Kaflik</maintainer>
      <license>MIT</license>

      <buildtool_depend>ament_cmake</buildtool_depend>

      <exec_depend>ros2launch</exec_depend>

      <depend>hardware_interface</depend>
      <depend>ros2_control</depend>

      <depend>rclcpp</depend>
      <depend>rclcpp_action</depend>
      <depend>action_msgs</depend>
      <depend>std_msgs</depend>
      <depend>std_srvs</depend>
      <depend>geometry_msgs</depend>
      <depend>sensor_msgs</depend>
      <depend>nav_msgs</depend>
      <depend>nav2_msgs</depend>
      <depend>tf2</depend>
      <depend>tf2_ros</depend>
      <depend>tf2_geometry_msgs</depend>
      <depend>ament_index_cpp</depend>
      <depend>visualization_msgs</depend>
      <depend>geographiclib</depend>
      <depend>opennav_docking</depend>
      <depend>opennav_docking_msgs</depend>

      <build_depend>nlohmann_json_schema_validator_vendor</build_depend>

      <exec_depend>vesc_hw_interface</exec_depend>
      <exec_depend>control_msgs</exec_depend>
      <exec_depend>controller_manager</exec_depend>
      <exec_depend>diff_drive_controller</exec_depend>
      <exec_depend>effort_controllers</exec_depend>
      <exec_depend>gz_ros2_control</exec_depend>
      <exec_depend>imu_sensor_broadcaster</exec_depend>
      <exec_depend>joint_state_broadcaster</exec_depend>
      <exec_depend>joint_trajectory_controller</exec_depend>
      <exec_depend>launch_ros</exec_depend>
      <exec_depend>launch</exec_depend>
      <exec_depend>robot_state_publisher</exec_depend>
      <exec_depend>ros_gz_sim</exec_depend>
      <exec_depend>ros_gz</exec_depend>
      <exec_depend>ros2controlcli</exec_depend>
      <exec_depend>ros2launch</exec_depend>
      <exec_depend>rviz2</exec_depend>
      <exec_depend>tricycle_controller</exec_depend>
      <exec_depend>velocity_controllers</exec_depend>
      <exec_depend>xacro</exec_depend>
      <exec_depend>joy</exec_depend>
      <exec_depend>teleop_twist_joy</exec_depend>
      <exec_depend>twist_mux</exec_depend>
      <exec_depend>rqt_robot_steering</exec_depend>
      <exec_depend>unique_identifier_msgs</exec_depend>

      <exec_depend>ublox_gps</exec_depend>
      <exec_depend>ntrip_client</exec_depend>

      <exec_depend>navigation2</exec_depend>
      <exec_depend>robot_localization</exec_depend>

      <exec_depend>foxglove_msgs</exec_depend>
      <exec_depend>foxglove_bridge</exec_depend>

      <test_depend>ament_lint_auto</test_depend>
      <test_depend>ament_lint_common</test_depend>

      <build_depend>rosidl_default_generators</build_depend>
      <exec_depend>rosidl_default_runtime</exec_depend>
      <member_of_group>rosidl_interface_packages</member_of_group>

      <export>
        <build_type>ament_cmake</build_type>
        <opennav_docking plugin="${prefix}/src/docking_helper/plugins.xml" />
      </export>
    </package>
    """,
    "CMakeLists.txt": """
    cmake_minimum_required(VERSION 3.8)
    project(open_mower_next)

    # Default to C++14
    if (NOT CMAKE_CXX_STANDARD)
        set(CMAKE_CXX_STANDARD 17)
    endif ()

    if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
    endif ()

    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(rclcpp_action REQUIRED)
    find_package(action_msgs REQUIRED)
    find_package(nlohmann_json REQUIRED)
    find_package(rosidl_default_generators REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(nav2_msgs REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(std_srvs REQUIRED)
    find_package(robot_localization REQUIRED)
    find_package(tf2 REQUIRED)
    find_package(tf2_ros REQUIRED)
    find_package(tf2_geometry_msgs REQUIRED)
    find_package(ament_index_cpp REQUIRED)
    find_package(foxglove_msgs REQUIRED)
    find_package(visualization_msgs REQUIRED)
    find_package(unique_identifier_msgs REQUIRED)
    find_package(pluginlib REQUIRED)
    find_package(opennav_docking REQUIRED)
    find_package(opennav_docking_msgs REQUIRED)

    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
    find_package(GeographicLib REQUIRED)

    rosidl_generate_interfaces(${PROJECT_NAME}
            "src/srv/RemoveArea.srv"
            "src/srv/RemoveDockingStation.srv"
            "src/srv/SaveArea.srv"
            "src/srv/SaveDockingStation.srv"
            "src/srv/FindNearestDockingStation.srv"
            "src/msg/Area.msg"
            "src/msg/Map.msg"
            "src/msg/DockingStation.msg"
            "src/action/DockRobotTo.action"
            "src/action/DockRobotNearest.action"
            "src/action/RecordDockingStation.action"
            "src/action/RecordAreaBoundary.action"
            DEPENDENCIES
            std_msgs
            action_msgs
            geometry_msgs
            sensor_msgs
            tf2
            tf2_geometry_msgs
            unique_identifier_msgs
    )

    ament_export_dependencies(rosidl_default_runtime)
    rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

    include(cmake/map_server.cmake)
    include(cmake/sim.cmake)
    include(cmake/docking_helper.cmake)
    include(cmake/map_recorder.cmake)

    if (BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        set(ament_cmake_copyright_FOUND TRUE)
        set(ament_cmake_cpplint_FOUND TRUE)
        ament_lint_auto_find_test_dependencies()
    endif ()

    INSTALL(
            DIRECTORY config launch description worlds maps
            DESTINATION share/${PROJECT_NAME}
    )

    ament_package()
    """,
    "description/robot.urdf.xacro": """
    <?xml version="1.0"?>
    <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="openmower">
        <xacro:arg name="use_ros2_control" default="true"/>
        <xacro:arg name="sim_mode" default="false"/>
        <xacro:arg name="hardware_config" default="$(find open_mower_next)/config/hardware/yardforce500.yaml"/>

        <xacro:property name="robot_yaml" value="$(arg hardware_config)" />
        <xacro:property name="robot" value="${xacro.load_yaml(robot_yaml)}"/>

        <xacro:include filename="robot_core.xacro" />
        <xacro:include filename="gps.xacro" />
        <xacro:include filename="imu.xacro" />
    <!--    <xacro:include filename="camera.xacro" />-->

        <xacro:if value="$(arg use_ros2_control)">
            <xacro:include filename="ros2_control.xacro" />
        </xacro:if>
        <xacro:unless value="$(arg use_ros2_control)">
            <xacro:include filename="gazebo_control.xacro" />
        </xacro:unless>
    </robot>
    """,
    "launch/gps.launch.py": """
    import os
    from launch import LaunchDescription
    from launch_ros.actions import Node
    from ament_index_python.packages import get_package_share_directory


    def generate_launch_description():
        config_dir = os.getenv(
            "OM_CONFIG_DIR",
            os.path.join(get_package_share_directory("open_mower_next"), "config"),
        )
        gps_params = os.getenv("OM_GPS_PARAMS", os.path.join(config_dir, "gps.yaml"))

        nodes = [
            Node(
                package="ublox_f9p",
                executable="ublox_f9p",
                output="both",
                parameters=[gps_params],
            ),
        ]

        if os.getenv("OM_NTRIP_ENABLED") == "true":
            nodes.append(
                Node(
                    name="ntrip_client_node",
                    package="ntrip_client",
                    executable="ntrip_ros.py",
                    parameters=[{
                        "host": os.getenv("OM_NTRIP_HOSTNAME", ""),
                        "port": int(os.getenv("OM_NTRIP_PORT", "2101")),
                        "mountpoint": os.getenv("OM_NTRIP_ENDPOINT", ""),
                        "authenticate": os.getenv("OM_NTRIP_AUTHENTICATE", "false") == "true",
                        "username": os.getenv("OM_NTRIP_USER", ""),
                        "password": os.getenv("OM_NTRIP_PASSWORD", ""),
                        "rtcm_message_package": "rtcm_msgs",
                    }],
                ),
            )

        return LaunchDescription(nodes)
    """,
    "launch/localization.launch.py": """
    import os

    from ament_index_python.packages import get_package_share_directory

    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node


    def generate_launch_description():
        package_path = get_package_share_directory("open_mower_next")
        config_dir = os.getenv("OM_CONFIG_DIR", os.path.join(package_path, "config"))

        use_sim_time = LaunchConfiguration("use_sim_time")
        use_external_hardware = os.getenv("OM_USE_EXTERNAL_HARDWARE", "true").lower() == "true"

        localization_params_path = os.getenv(
            "OM_LOCALIZATION_PARAMS",
            os.path.join(config_dir, "robot_localization.yaml"),
        )

        actions = [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            DeclareLaunchArgument(
                "namespace", default_value="", description="Top-level namespace"
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Automatically startup the nav2 stack",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(package_path, "config", "nav2_params.yaml"),
                description="Full path to the ROS2 parameters file to use",
            ),
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package="open_mower_next",
                        executable="map_server_node",
                        name="map_server",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": use_sim_time,
                                "path": os.getenv("OM_MAP_PATH"),
                                "datum": [
                                    float(os.getenv("OM_DATUM_LAT")),
                                    float(os.getenv("OM_DATUM_LONG")),
                                ],
                                "grid.use_gaussian_blur": True,
                            }
                        ],
                        remappings=[
                            ("map_grid", "map_grid"),
                            ("map", "mowing_map"),
                        ],
                    )
                ],
            ),
            Node(
                package="open_mower_next",
                executable="map_recorder",
                name="map_recorder",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package="open_mower_next",
                executable="docking_helper",
                name="docking_helper",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_se_odom",
                output="screen",
                parameters=[localization_params_path, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("imu/data", "imu/data_raw"),
                ],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_se_map",
                output="screen",
                parameters=[localization_params_path, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("imu/data", "imu/data_raw"),
                    ("odometry/filtered", "odometry/filtered/map"),
                ],
            ),
        ]

        if not use_external_hardware:
            actions.append(
                Node(
                    package="robot_localization",
                    executable="navsat_transform_node",
                    name="navsat_transform_node",
                    output="screen",
                    parameters=[
                        localization_params_path,
                        {
                            "use_sim_time": use_sim_time,
                            "datum": [
                                float(os.getenv("OM_DATUM_LAT")),
                                float(os.getenv("OM_DATUM_LONG")),
                                0.0,
                            ],
                        },
                    ],
                    remappings=[
                        ("odometry/filtered", "odometry/filtered/map"),
                        ("imu", "gps/orientation"),
                    ],
                )
            )

        return LaunchDescription(actions)
    """,
    "launch/nav2.launch.py": """
    import os
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import LoadComposableNodes
    from launch_ros.actions import Node
    from launch_ros.descriptions import ComposableNode, ParameterFile
    from nav2_common.launch import RewrittenYaml


    def generate_launch_description():
        config_dir = os.getenv(
            "OM_CONFIG_DIR",
            os.path.join(get_package_share_directory("open_mower_next"), "config"),
        )
        default_params_file = os.getenv(
            "OM_NAV2_PARAMS",
            os.path.join(config_dir, "nav2_params.yaml"),
        )

        lifecycle_nodes = [
            "controller_server",
            "planner_server",
            "behavior_server",
            "bt_navigator",
            "velocity_smoother",
            "docking_server",
            "smoother_server",
        ]

        remappings = [
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ]

        params_file = LaunchConfiguration("params_file")
        use_sim_time = LaunchConfiguration("use_sim_time")
        autostart = LaunchConfiguration("autostart")
        param_substitutions = {
            "use_sim_time": use_sim_time,
            "autostart": autostart,
        }

        configured_params = ParameterFile(
            RewrittenYaml(
                source_file=params_file,
                root_key="",
                param_rewrites=param_substitutions,
                convert_types=True,
            ),
            allow_substs=True,
        )

        stdout_linebuf_envvar = SetEnvironmentVariable(
            "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
        )

        create_container = Node(
            name="nav2_container",
            package="rclcpp_components",
            executable="component_container_isolated",
            parameters=[configured_params, {"autostart": autostart}],
            remappings=remappings,
            output="screen",
        )

        load_composable_nodes = LoadComposableNodes(
            target_container="nav2_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="nav2_controller",
                    plugin="nav2_controller::ControllerServer",
                    name="controller_server",
                    parameters=[configured_params],
                    remappings=remappings + [("cmd_vel", "cmd_vel_raw")],
                ),
                ComposableNode(
                    package="nav2_bt_navigator",
                    plugin="nav2_bt_navigator::BtNavigator",
                    name="bt_navigator",
                    parameters=[configured_params],
                    remappings=remappings,
                ),
                ComposableNode(
                    package="nav2_behaviors",
                    plugin="behavior_server::BehaviorServer",
                    name="behavior_server",
                    parameters=[configured_params],
                    remappings=remappings + [("cmd_vel", "cmd_vel_raw")],
                ),
                ComposableNode(
                    package="nav2_planner",
                    plugin="nav2_planner::PlannerServer",
                    name="planner_server",
                    parameters=[configured_params],
                    remappings=remappings,
                ),
                ComposableNode(
                    package="nav2_smoother",
                    plugin="nav2_smoother::SmootherServer",
                    name="smoother_server",
                    parameters=[configured_params],
                    remappings=remappings,
                ),
                ComposableNode(
                    package="nav2_velocity_smoother",
                    plugin="nav2_velocity_smoother::VelocitySmoother",
                    name="velocity_smoother",
                    parameters=[configured_params],
                    remappings=remappings
                    + [("cmd_vel", "cmd_vel_raw"), ("cmd_vel_smoothed", "cmd_vel_nav")],
                ),
                ComposableNode(
                    package="opennav_docking",
                    plugin="opennav_docking::DockingServer",
                    name="docking_server",
                    parameters=[configured_params],
                    remappings=remappings + [("cmd_vel", "cmd_vel_raw")],
                ),
                ComposableNode(
                    package="nav2_lifecycle_manager",
                    plugin="nav2_lifecycle_manager::LifecycleManager",
                    name="lifecycle_manager_navigation",
                    parameters=[{
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": lifecycle_nodes,
                    }],
                ),
            ],
        )

        return LaunchDescription([
            stdout_linebuf_envvar,
            DeclareLaunchArgument("params_file", default_value=default_params_file),
            DeclareLaunchArgument("use_sim_time", default_value="True"),
            DeclareLaunchArgument("autostart", default_value="True"),
            create_container,
            load_composable_nodes,
        ])
    """,
    "launch/micro_ros_agent.launch.py": """
    import os
    import yaml
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node
    from ament_index_python.packages import get_package_share_directory


    def _load_micro_ros_settings(path):
        if not path or not os.path.exists(path):
            return "/dev/ttyAMA0", "115200"

        with open(path, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}

        config = data.get("micro_ros_agent", {})
        return str(config.get("device", "/dev/ttyAMA0")), str(config.get("baudrate", "115200"))


    def generate_launch_description():
        config_dir = os.getenv(
            "OM_CONFIG_DIR",
            os.path.join(get_package_share_directory("open_mower_next"), "config"),
        )
        default_yaml_path = os.getenv(
            "OM_MICRO_ROS_CONFIG",
            os.path.join(config_dir, "hardware", "openmower.yaml"),
        )

        device, baudrate = _load_micro_ros_settings(default_yaml_path)

        return LaunchDescription([
            DeclareLaunchArgument(
                "yaml_file",
                default_value=default_yaml_path,
                description="Full path to the YAML file to use for this launch session",
            ),
            Node(
                package="micro_ros_agent",
                executable="micro_ros_agent",
                output="screen",
                parameters=[LaunchConfiguration("yaml_file")],
                arguments=["serial", "--dev", device, "--baud", baudrate],
            ),
        ])
    """,
    "launch/openmower.launch.py": """
    import os

    from ament_index_python.packages import get_package_share_directory

    from launch import LaunchDescription
    from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
    from launch.event_handlers import OnProcessStart, OnProcessExit
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

    from launch_ros.actions import Node

    import xacro


    def _env_flag(name, default):
        return os.getenv(name, default).lower() == "true"


    def generate_launch_description():
        package_name = "open_mower_next"

        share_directory = get_package_share_directory(package_name)
        config_dir = os.getenv("OM_CONFIG_DIR", os.path.join(share_directory, "config"))
        hardware_config = os.getenv(
            "OM_HARDWARE_CONFIG",
            os.path.join(config_dir, "hardware", "yardforce500.yaml"),
        )
        controllers_config = os.getenv(
            "OM_CONTROLLERS_CONFIG",
            os.path.join(config_dir, "controllers.yaml"),
        )
        twist_mux_config = os.getenv(
            "OM_TWIST_MUX_CONFIG",
            os.path.join(config_dir, "twist_mux.yaml"),
        )
        use_sim_time = _env_flag("OM_USE_SIM_TIME", "false")
        autostart = os.getenv("OM_AUTOSTART", "true").lower()
        enable_foxglove = _env_flag("OM_ENABLE_FOXGLOVE", "false")
        use_external_hardware = _env_flag("OM_USE_EXTERNAL_HARDWARE", "true")
        cmd_vel_output_topic = os.getenv("OM_EXTERNAL_CMD_VEL_TOPIC", "/cmd_vel")

        xacro_file = os.path.join(share_directory, "description/robot.urdf.xacro")
        robot_description_config = xacro.process_file(xacro_file, mappings={
            "use_ros2_control": "0" if use_external_hardware else "1",
            "use_sim_time": "1" if use_sim_time else "0",
            "hardware_config": hardware_config,
        }).toxml()

        params = {"robot_description": robot_description_config, "use_sim_time": use_sim_time}
        node_robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[params],
        )

        joystick = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(share_directory, "launch", "joystick.launch.py")
            ]),
            launch_arguments={"use_sim_time": str(use_sim_time).lower()}.items(),
        )

        twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_config, {"use_sim_time": use_sim_time}],
            remappings=[(
                "/cmd_vel_out",
                cmd_vel_output_topic if use_external_hardware else "/diff_drive_base_controller/cmd_vel",
            )],
        )

        controller_manager = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[{"robot_description": robot_description_config}, controllers_config],
        )

        load_joint_state_controller = ExecuteProcess(
            cmd=["ros2", "control", "load_controller", "--set-state", "active", "joint_state_broadcaster"],
            output="screen",
        )

        load_diff_controller = ExecuteProcess(
            cmd=["ros2", "control", "load_controller", "--set-state", "active", "diff_drive_base_controller"],
            output="screen",
        )

        load_mower_controller = ExecuteProcess(
            cmd=["ros2", "control", "load_controller", "--set-state", "active", "mower_controller"],
            output="screen",
        )

        actions = [
            node_robot_state_publisher,
            joystick,
            twist_mux,
        ]

        if not use_external_hardware:
            actions.extend([
                controller_manager,
                RegisterEventHandler(
                    event_handler=OnProcessStart(
                        target_action=controller_manager,
                        on_start=[load_joint_state_controller],
                    )
                ),
                RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=load_joint_state_controller,
                        on_exit=[load_diff_controller],
                    )
                ),
                RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=load_joint_state_controller,
                        on_exit=[load_mower_controller],
                    )
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([share_directory, "/launch/gps.launch.py"]),
                ),
            ])

        actions.extend([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([share_directory, "/launch/localization.launch.py"]),
                launch_arguments={
                    "use_sim_time": str(use_sim_time).lower(),
                    "autostart": autostart,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([share_directory, "/launch/nav2.launch.py"]),
                launch_arguments={
                    "params_file": os.getenv(
                        "OM_NAV2_PARAMS",
                        os.path.join(config_dir, "nav2_params.yaml"),
                    ),
                    "use_sim_time": str(use_sim_time).lower(),
                    "autostart": autostart,
                }.items(),
            ),
        ])

        if enable_foxglove:
            actions.append(
                IncludeLaunchDescription(
                    XMLLaunchDescriptionSource([
                        get_package_share_directory("foxglove_bridge"),
                        "/launch/foxglove_bridge_launch.xml",
                    ]),
                )
            )

        return LaunchDescription(actions)
    """,
    "cmake/docking_helper.cmake": """
    ###
    # docking_helper
    ###
    add_executable(docking_helper
            src/docking_helper/main.cpp
            src/docking_helper/docking_helper_node.cpp)
    target_compile_features(docking_helper PUBLIC c_std_99 cxx_std_17)
    target_include_directories(docking_helper PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
            $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(docking_helper "${cpp_typesupport_target}")

    ament_target_dependencies(docking_helper
            rclcpp
            rclcpp_action
            tf2
            tf2_ros
            tf2_geometry_msgs
            opennav_docking_msgs
            geometry_msgs
            std_msgs
    )

    INSTALL(TARGETS docking_helper
            DESTINATION lib/${PROJECT_NAME})

    add_library(charger_presence_charging_dock SHARED
            src/docking_helper/charger_presence_charging_dock.cpp)
    target_compile_features(charger_presence_charging_dock PUBLIC c_std_99 cxx_std_17)
    target_include_directories(charger_presence_charging_dock PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
            $<INSTALL_INTERFACE:include>
    )

    ament_target_dependencies(charger_presence_charging_dock
      pluginlib
      opennav_docking_core
      std_msgs
      rclcpp_lifecycle
      tf2_ros
    )

    INSTALL(TARGETS charger_presence_charging_dock
            DESTINATION lib/${PROJECT_NAME})

    INSTALL(FILES src/docking_helper/charger_presence_charging_dock.hpp
            DESTINATION include/${PROJECT_NAME}/docking_helper)

    set_target_properties(charger_presence_charging_dock PROPERTIES LINKER_LANGUAGE CXX)
    pluginlib_export_plugin_description_file(opennav_docking_core src/docking_helper/plugins.xml)

    add_dependencies(docking_helper ${PROJECT_NAME})
    """,
    "src/docking_helper/docking_helper_node.hpp": """
    #pragma once

    #include <memory>
    #include <mutex>
    #include <string>
    #include <vector>

    #include <geometry_msgs/msg/pose_stamped.hpp>
    #include <opennav_docking_msgs/action/dock_robot.hpp>
    #include <rclcpp/rclcpp.hpp>
    #include <rclcpp_action/rclcpp_action.hpp>
    #include <tf2_ros/buffer.h>
    #include <tf2_ros/transform_listener.h>

    #include "open_mower_next/action/dock_robot_nearest.hpp"
    #include "open_mower_next/action/dock_robot_to.hpp"
    #include "open_mower_next/msg/docking_station.hpp"
    #include "open_mower_next/msg/map.hpp"
    #include "open_mower_next/srv/find_nearest_docking_station.hpp"

    namespace open_mower_next::docking_helper
    {
    class DockingHelperNode : public rclcpp::Node
    {
    public:
      explicit DockingHelperNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
      ~DockingHelperNode();

    private:
      using DockRobotAction = opennav_docking_msgs::action::DockRobot;

      using DockRobotNearestAction = open_mower_next::action::DockRobotNearest;
      using DockRobotNearestGoalHandle = rclcpp_action::ServerGoalHandle<DockRobotNearestAction>;

      using DockRobotToAction = open_mower_next::action::DockRobotTo;
      using DockRobotToGoalHandle = rclcpp_action::ServerGoalHandle<DockRobotToAction>;

      std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
      std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

      void mapCallback(const open_mower_next::msg::Map::SharedPtr msg);
      std::shared_ptr<rclcpp::Subscription<open_mower_next::msg::Map>> map_sub_;
      std::mutex docking_stations_mutex_;
      std::vector<open_mower_next::msg::DockingStation> docking_stations_;

      std::shared_ptr<open_mower_next::msg::DockingStation> findNearestDockingStation();
      void findNearestDockingStationService(
          const std::shared_ptr<open_mower_next::srv::FindNearestDockingStation::Request> request,
          std::shared_ptr<open_mower_next::srv::FindNearestDockingStation::Response> response);

      std::shared_ptr<geometry_msgs::msg::PoseStamped>
      dockPose(const std::shared_ptr<open_mower_next::msg::DockingStation>& station);

      rclcpp::Service<open_mower_next::srv::FindNearestDockingStation>::SharedPtr find_nearest_docking_station_service_;

      rclcpp_action::Client<DockRobotAction>::SharedPtr dock_client_;

      rclcpp_action::Server<DockRobotNearestAction>::SharedPtr dock_robot_nearest_server_;
      rclcpp_action::Server<DockRobotToAction>::SharedPtr dock_robot_to_server_;

      rclcpp_action::GoalResponse handleDockRobotNearestGoal(const rclcpp_action::GoalUUID& uuid,
                                                             std::shared_ptr<const DockRobotNearestAction::Goal> goal);
      rclcpp_action::CancelResponse
      handleDockRobotNearestCancel(const std::shared_ptr<DockRobotNearestGoalHandle> goal_handle);
      void handleDockRobotNearestAccepted(const std::shared_ptr<DockRobotNearestGoalHandle> goal_handle);

      rclcpp_action::GoalResponse handleDockRobotToGoal(const rclcpp_action::GoalUUID& uuid,
                                                        std::shared_ptr<const DockRobotToAction::Goal> goal);
      rclcpp_action::CancelResponse handleDockRobotToCancel(const std::shared_ptr<DockRobotToGoalHandle> goal_handle);
      void handleDockRobotToAccepted(const std::shared_ptr<DockRobotToGoalHandle> goal_handle);

      std::shared_ptr<open_mower_next::msg::DockingStation> findDockingStationById(const std::string& id);

      template <typename ActionT, typename GoalHandleT>
      void executeDockingAction(const std::shared_ptr<GoalHandleT>& goal_handle,
                                const std::shared_ptr<open_mower_next::msg::DockingStation>& docking_station);
    };

    }  // namespace open_mower_next::docking_helper
    """,
    "src/docking_helper/docking_helper_node.cpp": """
    #include "docking_helper/docking_helper_node.hpp"

    #include <atomic>
    #include <chrono>
    #include <cmath>
    #include <functional>
    #include <limits>
    #include <thread>

    #include <geometry_msgs/msg/transform_stamped.hpp>
    #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

    using namespace std::placeholders;

    open_mower_next::docking_helper::DockingHelperNode::DockingHelperNode(const rclcpp::NodeOptions& options)
      : Node("docking_helper", options)
    {
      map_sub_ = create_subscription<open_mower_next::msg::Map>(
          "/mowing_map", rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::TransientLocal),
          std::bind(&DockingHelperNode::mapCallback, this, std::placeholders::_1));

      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

      find_nearest_docking_station_service_ = create_service<open_mower_next::srv::FindNearestDockingStation>(
          "/find_nearest_docking_station", std::bind(&DockingHelperNode::findNearestDockingStationService, this,
                                                     std::placeholders::_1, std::placeholders::_2));

      dock_client_ = rclcpp_action::create_client<DockRobotAction>(this, "/dock_robot");

      dock_robot_nearest_server_ = rclcpp_action::create_server<DockRobotNearestAction>(
          this, "dock_robot_nearest", std::bind(&DockingHelperNode::handleDockRobotNearestGoal, this, _1, _2),
          std::bind(&DockingHelperNode::handleDockRobotNearestCancel, this, _1),
          std::bind(&DockingHelperNode::handleDockRobotNearestAccepted, this, _1));

      dock_robot_to_server_ = rclcpp_action::create_server<DockRobotToAction>(
          this, "dock_robot_to", std::bind(&DockingHelperNode::handleDockRobotToGoal, this, _1, _2),
          std::bind(&DockingHelperNode::handleDockRobotToCancel, this, _1),
          std::bind(&DockingHelperNode::handleDockRobotToAccepted, this, _1));
    }

    open_mower_next::docking_helper::DockingHelperNode::~DockingHelperNode()
    {
    }

    void open_mower_next::docking_helper::DockingHelperNode::mapCallback(const open_mower_next::msg::Map::SharedPtr msg)
    {
      RCLCPP_INFO(get_logger(), "Received map with %zu docking stations", msg->docking_stations.size());

      {
        std::lock_guard<std::mutex> lock(docking_stations_mutex_);
        docking_stations_.clear();

        for (const auto& docking_station : msg->docking_stations)
        {
          RCLCPP_DEBUG(get_logger(), "Docking station: %s", docking_station.name.c_str());

          docking_stations_.push_back(docking_station);
        }
      }
    }

    std::shared_ptr<open_mower_next::msg::DockingStation>
    open_mower_next::docking_helper::DockingHelperNode::findNearestDockingStation()
    {
      std::lock_guard<std::mutex> lock(docking_stations_mutex_);

      if (docking_stations_.empty())
      {
        RCLCPP_ERROR(get_logger(), "No docking stations available");
        return nullptr;
      }

      geometry_msgs::msg::PoseStamped robot_pose;
      geometry_msgs::msg::TransformStamped transform;

      try
      {
        transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

        robot_pose.header.frame_id = "map";
        robot_pose.header.stamp = this->now();
        robot_pose.pose.position.x = transform.transform.translation.x;
        robot_pose.pose.position.y = transform.transform.translation.y;
        robot_pose.pose.position.z = transform.transform.translation.z;
        robot_pose.pose.orientation = transform.transform.rotation;
      }
      catch (const tf2::TransformException& ex)
      {
        RCLCPP_ERROR(get_logger(), "Could not transform pose: %s", ex.what());
        return nullptr;
      }

      double min_distance = std::numeric_limits<double>::max();
      open_mower_next::msg::DockingStation nearest_station;

      for (const auto& station : docking_stations_)
      {
        double dx = station.pose.pose.position.x - robot_pose.pose.position.x;
        double dy = station.pose.pose.position.y - robot_pose.pose.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        if (distance < min_distance)
        {
          min_distance = distance;
          nearest_station = station;
        }
      }

      RCLCPP_INFO(get_logger(), "Found nearest docking station at distance: %f meters", min_distance);
      return std::make_shared<open_mower_next::msg::DockingStation>(nearest_station);
    }

    void open_mower_next::docking_helper::DockingHelperNode::findNearestDockingStationService(
        const std::shared_ptr<open_mower_next::srv::FindNearestDockingStation::Request> request,
        std::shared_ptr<open_mower_next::srv::FindNearestDockingStation::Response> response)
    {
      (void)request;
      try
      {
        auto nearest_station = findNearestDockingStation();

        if (!nearest_station)
        {
          response->code = open_mower_next::srv::FindNearestDockingStation::Response::CODE_NOT_FOUND;
          RCLCPP_ERROR(get_logger(), "Failed to find nearest docking station");
          return;
        }

        response->docking_station = *nearest_station;
        response->code = open_mower_next::srv::FindNearestDockingStation::Response::CODE_SUCCESS;
      }
      catch (const std::exception& e)
      {
        RCLCPP_ERROR(get_logger(), "Exception occured while finding nearest docking station: %s", e.what());
        response->code = open_mower_next::srv::FindNearestDockingStation::Response::CODE_UNKNOWN_ERROR;
        return;
      }
    }

    std::shared_ptr<geometry_msgs::msg::PoseStamped> open_mower_next::docking_helper::DockingHelperNode::dockPose(
        const std::shared_ptr<open_mower_next::msg::DockingStation>& station)
    {
      if (!station)
      {
        RCLCPP_ERROR(get_logger(), "Cannot transform null docking station");
        return nullptr;
      }

      auto pose_stamped = std::make_shared<geometry_msgs::msg::PoseStamped>();
      pose_stamped->header = station->pose.header;
      pose_stamped->pose = station->pose.pose;

      geometry_msgs::msg::TransformStamped transform_stamped =
          tf_buffer_->lookupTransform("base_link", "charging_port", tf2::TimePointZero);

      double offset_distance =
          std::sqrt(transform_stamped.transform.translation.x * transform_stamped.transform.translation.x +
                    transform_stamped.transform.translation.y * transform_stamped.transform.translation.y);

      RCLCPP_INFO(get_logger(), "Calculated offset distance from base_link to charging_port: %f", offset_distance);

      tf2::Quaternion q_orig, q_rot, q_new;
      tf2::fromMsg(pose_stamped->pose.orientation, q_orig);
      q_rot.setRPY(0.0, 0.0, M_PI);
      q_new = q_orig * q_rot;
      q_new.normalize();

      pose_stamped->pose.orientation = tf2::toMsg(q_new);

      tf2::Vector3 offset(offset_distance, 0.0, 0.0);
      tf2::Transform transform;
      transform.setRotation(q_new);
      tf2::Vector3 translated_offset = transform * offset;

      pose_stamped->pose.position.x -= translated_offset.x();
      pose_stamped->pose.position.y -= translated_offset.y();
      pose_stamped->pose.position.z -= translated_offset.z();

      return pose_stamped;
    }

    rclcpp_action::GoalResponse open_mower_next::docking_helper::DockingHelperNode::handleDockRobotNearestGoal(
        const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const DockRobotNearestAction::Goal> goal)
    {
      (void)uuid;
      (void)goal;
      RCLCPP_INFO(get_logger(), "Received request to dock to nearest docking station");

      {
        std::lock_guard<std::mutex> lock(docking_stations_mutex_);
        if (docking_stations_.empty())
        {
          RCLCPP_ERROR(get_logger(), "No docking stations available");
          return rclcpp_action::GoalResponse::REJECT;
        }
      }

      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse open_mower_next::docking_helper::DockingHelperNode::handleDockRobotNearestCancel(
        const std::shared_ptr<DockRobotNearestGoalHandle> goal_handle)
    {
      (void)goal_handle;
      RCLCPP_INFO(get_logger(), "Received request to cancel docking to nearest station");
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    template <typename ActionT, typename GoalHandleT>
    void open_mower_next::docking_helper::DockingHelperNode::executeDockingAction(
        const std::shared_ptr<GoalHandleT>& goal_handle,
        const std::shared_ptr<open_mower_next::msg::DockingStation>& docking_station)
    {
      auto feedback = std::make_shared<typename ActionT::Feedback>();
      auto result = std::make_shared<typename ActionT::Result>();

      feedback->status = ActionT::Feedback::STATUS_NONE;
      feedback->num_retries = 0;
      feedback->docking_time.sec = 0;
      feedback->docking_time.nanosec = 0;

      if (!docking_station)
      {
        result->code = ActionT::Result::CODE_DOCK_NOT_IN_DB;
        result->message = "No docking station available";
        result->num_retries = 0;
        goal_handle->abort(result);
        return;
      }

      feedback->chosen_docking_station = *docking_station;
      result->chosen_docking_station = *docking_station;

      feedback->status = ActionT::Feedback::STATUS_NAV_TO_STAGING_POSE;
      feedback->message = "Starting docking to: " + docking_station->name;
      goal_handle->publish_feedback(feedback);

      auto start_time = this->now();

      std::shared_ptr<uint16_t> current_status = std::make_shared<uint16_t>(ActionT::Feedback::STATUS_NONE);
      std::shared_ptr<uint16_t> current_retries = std::make_shared<uint16_t>(0);
      std::atomic<bool> docking_active(true);

      auto nav2_goal = DockRobotAction::Goal();
      nav2_goal.use_dock_id = false;
      nav2_goal.navigate_to_staging_pose = true;
      nav2_goal.dock_pose.header = docking_station->pose.header;
      nav2_goal.dock_pose.pose = dockPose(docking_station)->pose;

      if (!dock_client_->wait_for_action_server(std::chrono::seconds(5)))
      {
        result->code = ActionT::Result::CODE_UNKNOWN;
        result->message = "Dock robot action server not available";
        result->num_retries = 0;
        goal_handle->abort(result);
        return;
      }

      RCLCPP_INFO(get_logger(), "Sending docking goal to station: %s", docking_station->name.c_str());
      auto send_goal_options = rclcpp_action::Client<DockRobotAction>::SendGoalOptions();

      send_goal_options.feedback_callback =
          [this, current_status, current_retries](typename rclcpp_action::ClientGoalHandle<DockRobotAction>::SharedPtr,
                                                  const std::shared_ptr<const DockRobotAction::Feedback> feedback) {
            RCLCPP_INFO(get_logger(), "Docking state: %d, retries: %d", feedback->state, feedback->num_retries);

            *current_status = feedback->state;
            *current_retries = feedback->num_retries;
          };

      send_goal_options.goal_response_callback = [this](const auto& goal_handle) {
        if (!goal_handle)
        {
          RCLCPP_ERROR(get_logger(), "Docking goal was rejected by server");
        }
        else
        {
          RCLCPP_INFO(get_logger(), "Docking goal accepted by server");
        }
      };

      send_goal_options.result_callback = [this, goal_handle, result, &docking_active](const auto& nav_result) {
        auto status = nav_result.result;
        bool success = status->success;
        uint16_t error_code = status->error_code;
        uint16_t num_retries = status->num_retries;

        docking_active = false;

        result->num_retries = num_retries;

        if (success)
        {
          RCLCPP_INFO(get_logger(), "Docking action succeeded");
          result->code = ActionT::Result::CODE_SUCCESS;
          result->message = "Docking completed successfully";
          goal_handle->succeed(result);
        }
        else
        {
          RCLCPP_ERROR(get_logger(), "Docking action failed with error code: %d", error_code);
          result->code = error_code;
          result->message = "Docking failed with error code: " + std::to_string(error_code);
          goal_handle->abort(result);
        }
      };

      dock_client_->async_send_goal(nav2_goal, send_goal_options);

      std::string status_messages[] = { "No activity",         "Navigating to staging pose", "Initial perception of dock",
                                        "Controlling to dock", "Waiting for charge",         "Retrying docking" };

      uint16_t last_status = 99;
      uint16_t last_retries = 0;

      while (docking_active && rclcpp::ok())
      {
        auto current_time = this->now();
        auto elapsed = current_time - start_time;
        feedback->docking_time.sec = elapsed.seconds();
        feedback->docking_time.nanosec = elapsed.nanoseconds() % 1000000000;

        uint16_t status = *current_status;
        uint16_t retries = *current_retries;

        if (status != last_status || retries != last_retries)
        {
          feedback->status = status;
          feedback->num_retries = retries;

          if (status < sizeof(status_messages) / sizeof(status_messages[0]))
          {
            feedback->message = status_messages[status];
            if (retries > 0)
            {
              feedback->message += " (retry " + std::to_string(retries) + ")";
            }
          }
          else
          {
            feedback->message = "Unknown status: " + std::to_string(status);
          }

          last_status = status;
          last_retries = retries;

          goal_handle->publish_feedback(feedback);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }

    void open_mower_next::docking_helper::DockingHelperNode::handleDockRobotNearestAccepted(
        const std::shared_ptr<DockRobotNearestGoalHandle> goal_handle)
    {
      std::thread{ [this, goal_handle]() {
        auto nearest_station = findNearestDockingStation();
        executeDockingAction<DockRobotNearestAction>(goal_handle, nearest_station);
      } }.detach();
    }

    std::shared_ptr<open_mower_next::msg::DockingStation>
    open_mower_next::docking_helper::DockingHelperNode::findDockingStationById(const std::string& id)
    {
      std::lock_guard<std::mutex> lock(docking_stations_mutex_);

      for (const auto& station : docking_stations_)
      {
        if (station.id == id)
        {
          return std::make_shared<open_mower_next::msg::DockingStation>(station);
        }
      }
      return nullptr;
    }

    rclcpp_action::GoalResponse open_mower_next::docking_helper::DockingHelperNode::handleDockRobotToGoal(
        const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const DockRobotToAction::Goal> goal)
    {
      (void)uuid;
      RCLCPP_INFO(get_logger(), "Received request to dock to station ID: %s", goal->dock_id.c_str());

      {
        std::lock_guard<std::mutex> lock(docking_stations_mutex_);
        if (docking_stations_.empty())
        {
          RCLCPP_ERROR(get_logger(), "No docking stations available");
          return rclcpp_action::GoalResponse::REJECT;
        }
      }

      auto docking_station = findDockingStationById(goal->dock_id);
      if (!docking_station)
      {
        RCLCPP_ERROR(get_logger(), "Docking station with ID %s not found", goal->dock_id.c_str());
        return rclcpp_action::GoalResponse::REJECT;
      }

      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse open_mower_next::docking_helper::DockingHelperNode::handleDockRobotToCancel(
        const std::shared_ptr<DockRobotToGoalHandle> goal_handle)
    {
      RCLCPP_INFO(get_logger(), "Received request to cancel docking to station ID: %s",
                  goal_handle->get_goal()->dock_id.c_str());
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    void open_mower_next::docking_helper::DockingHelperNode::handleDockRobotToAccepted(
        const std::shared_ptr<DockRobotToGoalHandle> goal_handle)
    {
      std::thread{ [this, goal_handle]() {
        auto goal = goal_handle->get_goal();
        auto docking_station = findDockingStationById(goal->dock_id);
        executeDockingAction<DockRobotToAction>(goal_handle, docking_station);
      } }.detach();
    }
    """,
    "src/sim/sim_node.cpp": """
    #include "sim_node.hpp"

    #include <algorithm>
    #include <cmath>

    #include <geometry_msgs/msg/transform.hpp>
    #include <tf2/LinearMath/Transform.h>
    #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

    open_mower_next::sim::SimNode::SimNode(const rclcpp::NodeOptions& options) : Node("sim_node", options)
    {
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      docking_station_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
          "/model/docking_station/pose", 10, std::bind(&SimNode::dockingStationPoseCallback, this, std::placeholders::_1));

      charger_present_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/power/charger_present", 10);
      battery_state_publisher_ = this->create_publisher<sensor_msgs::msg::BatteryState>("/power", 10);
      charge_voltage_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/power/charge_voltage", 10);

      const auto freq_hz = static_cast<int>(this->declare_parameter<int32_t>("charger_simulation_freq", 15));

      battery_state_voltage_drop_per_second_ =
          this->declare_parameter<double>("battery_voltage_drop_per_second", 0.005);
      battery_state_voltage_charge_per_second_ =
          this->declare_parameter<double>("battery_voltage_charge_per_second", 0.01);

      battery_state_max_voltage_ = this->declare_parameter<double>("battery_max_voltage", 28.7);
      battery_state_min_voltage_ = this->declare_parameter<double>("battery_min_voltage", 21.7);

      battery_state_msg_.voltage =
          this->declare_parameter<double>("initial_battery_voltage", battery_state_max_voltage_);
      battery_state_msg_.design_capacity = this->declare_parameter<double>("battery_design_capacity", 2.0);
      battery_state_msg_.capacity =
          this->declare_parameter<double>("initial_battery_capacity", battery_state_msg_.design_capacity);

      battery_state_msg_.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
      battery_state_msg_.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
      battery_state_msg_.present = true;

      charger_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / freq_hz),
                                               [this] { chargerPresentSimulationCallback(); });

      battery_timer_ = this->create_wall_timer(std::chrono::seconds(1), [this] { batteryStateSimulationCallback(); });

      docking_station_offset_x_ = this->declare_parameter<double>("docking_station_offset_x", 0.32);
      docking_station_offset_y_ = this->declare_parameter<double>("docking_station_offset_y", 0.0);

      RCLCPP_INFO(get_logger(), "Docking station pose offset: x=%f, y=%f", docking_station_offset_x_,
                  docking_station_offset_y_);

      RCLCPP_INFO(get_logger(), "SimNode created with timer frequency: %d Hz", freq_hz);
    }

    bool open_mower_next::sim::SimNode::isInDockingStation()
    {
      if (!docking_station_pose_)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No docking station pose received yet");
        return false;
      }

      geometry_msgs::msg::TransformStamped charging_port_transform;
      try
      {
        charging_port_transform = tf_buffer_->lookupTransform("map", "charging_port", tf2::TimePointZero);
      }
      catch (const tf2::TransformException& ex)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Could not get charging port transform: %s", ex.what());
        return false;
      }

      geometry_msgs::msg::Pose charging_port_pose;
      tf2::Transform tf_charging_port;
      tf2::fromMsg(charging_port_transform.transform, tf_charging_port);
      tf2::toMsg(tf_charging_port, charging_port_pose);

      tf2::Transform port_transform, dock_transform;
      tf2::fromMsg(charging_port_pose, port_transform);
      tf2::fromMsg(docking_station_pose_->pose, dock_transform);

      auto relativeTransform = port_transform.inverseTimes(dock_transform);
      auto translation = relativeTransform.getOrigin();

      bool inDockingStation = std::abs(translation.x()) < 0.05 && std::abs(translation.y()) < 0.05;
      double distance = std::sqrt(std::pow(translation.x(), 2) + std::pow(translation.y(), 2));
      double angle = std::atan2(translation.y(), translation.x());

      if (!inDockingStation && distance < 1.0)
      {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Distance from charging port to docking station: %f m, angle: %f rad", distance, angle);
      }

      if (inDockingStation)
      {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Charging port in contact with docking station");
        return true;
      }

      return false;
    }

    void open_mower_next::sim::SimNode::chargerPresentSimulationCallback()
    {
      auto inDock = isInDockingStation();

      charger_present_msg_.data = inDock;
      charger_present_publisher_->publish(charger_present_msg_);

      if (inDock)
      {
        charge_voltage_msg_.data = battery_state_max_voltage_;
      }
      else
      {
        charge_voltage_msg_.data = 0.0;
      }

      charge_voltage_publisher_->publish(charge_voltage_msg_);
    }

    void open_mower_next::sim::SimNode::batteryStateSimulationCallback()
    {
      const auto now = get_clock()->now();

      if (last_battery_voltage_update_.seconds() == 0)
      {
        last_battery_voltage_update_ = now;
      }

      if (!charger_present_msg_.data)
      {
        auto sinceLastVoltageUpdate = (now - last_battery_voltage_update_).seconds();
        if (sinceLastVoltageUpdate >= 1)
        {
          battery_state_msg_.voltage -= sinceLastVoltageUpdate * battery_state_voltage_drop_per_second_;
        }
      }
      else
      {
        battery_state_msg_.voltage =
            std::min(battery_state_max_voltage_, battery_state_msg_.voltage + battery_state_voltage_charge_per_second_);
      }

      last_battery_voltage_update_ = now;
      battery_state_msg_.percentage = (battery_state_msg_.voltage - battery_state_min_voltage_) /
                                      (battery_state_max_voltage_ - battery_state_min_voltage_) * 100.0;

      if (charger_present_msg_.data)
      {
        if (battery_state_msg_.percentage < 100.0)
        {
          battery_state_msg_.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
        }
        else
        {
          battery_state_msg_.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
        }
      }

      battery_state_msg_.power_supply_health =
          battery_state_msg_.present ? (battery_state_msg_.voltage < battery_state_min_voltage_ ?
                                            sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_DEAD :
                                            (battery_state_msg_.voltage > battery_state_max_voltage_ ?
                                                 sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_OVERVOLTAGE :
                                                 sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD)) :
                                       sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;

      battery_state_msg_.header.stamp = now;
      battery_state_publisher_->publish(battery_state_msg_);
    }

    void open_mower_next::sim::SimNode::dockingStationPoseCallback(geometry_msgs::msg::Pose::SharedPtr msg)
    {
      RCLCPP_INFO_ONCE(get_logger(), "Received docking station pose message");

      if (!docking_station_pose_)
      {
        docking_station_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
      }

      docking_station_pose_->header.stamp = this->get_clock()->now();
      docking_station_pose_->pose = *msg;

      tf2::Transform dockTransform;
      tf2::fromMsg(*msg, dockTransform);

      tf2::Transform offsetTransform;
      offsetTransform.setOrigin(tf2::Vector3(docking_station_offset_x_, docking_station_offset_y_, 0.0));
      offsetTransform.setRotation(tf2::Quaternion(0, 0, 0, 1));

      tf2::Transform adjustedTransform = dockTransform * offsetTransform;
      tf2::toMsg(adjustedTransform, docking_station_pose_->pose);

      RCLCPP_INFO_ONCE(get_logger(), "Original docking station pose: [%f, %f, %f]", msg->position.x, msg->position.y,
                       msg->position.z);
      RCLCPP_INFO_ONCE(get_logger(), "Adjusted docking station pose: [%f, %f, %f]", docking_station_pose_->pose.position.x,
                       docking_station_pose_->pose.position.y, docking_station_pose_->pose.position.z);
    }
    """,
    "src/docking_helper/charger_presence_charging_dock.hpp": """
    #pragma once

    #include <rclcpp/rclcpp.hpp>
    #include <rclcpp_lifecycle/lifecycle_node.hpp>
    #include <opennav_docking_core/charging_dock.hpp>
    #include <std_msgs/msg/bool.hpp>
    #include <tf2/utils.h>
    #include <geometry_msgs/msg/pose_stamped.hpp>

    namespace open_mower_next::docking_helper
    {

    class ChargerPresenceChargingDock : public opennav_docking_core::ChargingDock
    {
    public:
      ChargerPresenceChargingDock();

      void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, const std::string& name,
                     std::shared_ptr<tf2_ros::Buffer> tf) override;

      void cleanup() override;
      void activate() override;
      void deactivate() override;

      geometry_msgs::msg::PoseStamped getStagingPose(const geometry_msgs::msg::Pose& pose,
                                                     const std::string& frame) override;

      bool getRefinedPose(geometry_msgs::msg::PoseStamped& pose) override;
      bool isDocked() override;
      bool isCharging() override;
      bool disableCharging() override;
      bool hasStoppedCharging() override;

    private:
      rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_charging_sub_;
      bool is_charging_ = false;
      rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
      std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

      rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr dock_pose_pub_;
      rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr staging_pose_pub_;

      double staging_x_offset_ = 0.5;
      double staging_yaw_offset_ = 0.0;
    };

    }  // namespace open_mower_next::docking_helper
    """,
    "src/docking_helper/charger_presence_charging_dock.cpp": """
    #include "docking_helper/charger_presence_charging_dock.hpp"
    #include "pluginlib/class_list_macros.hpp"
    #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

    namespace open_mower_next::docking_helper
    {

    ChargerPresenceChargingDock::ChargerPresenceChargingDock() : opennav_docking_core::ChargingDock()
    {
    }

    void ChargerPresenceChargingDock::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
                                                const std::string& name, std::shared_ptr<tf2_ros::Buffer> tf_buffer)
    {
      auto node = parent.lock();
      if (!node)
      {
        throw std::runtime_error("Unable to lock weak_ptr to LifecycleNode in configure()");
      }

      node_ = node;
      tf_buffer_ = tf_buffer;

      node_->declare_parameter(name + ".staging_x_offset", 1.0);
      node_->declare_parameter(name + ".staging_yaw_offset", 0.0);

      staging_x_offset_ = node_->get_parameter(name + ".staging_x_offset").as_double();
      staging_yaw_offset_ = node_->get_parameter(name + ".staging_yaw_offset").as_double();

      is_charging_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
          "/power/charger_present", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) { is_charging_ = msg->data; });

      dock_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("dock_pose", 1);
      staging_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("staging_pose", 1);

      RCLCPP_INFO(node_->get_logger(),
                  "ChargerPresenceChargingDock configured with staging_x_offset: %f, staging_yaw_offset: %f",
                  staging_x_offset_, staging_yaw_offset_);
    }

    void ChargerPresenceChargingDock::cleanup()
    {
      is_charging_sub_.reset();
    }

    void ChargerPresenceChargingDock::activate()
    {
    }

    void ChargerPresenceChargingDock::deactivate()
    {
    }

    geometry_msgs::msg::PoseStamped ChargerPresenceChargingDock::getStagingPose(const geometry_msgs::msg::Pose& pose,
                                                                                const std::string& frame)
    {
      geometry_msgs::msg::PoseStamped staging_pose;
      staging_pose.header.frame_id = frame;
      staging_pose.header.stamp = node_->now();

      staging_pose.pose = pose;

      tf2::Transform pose_tf;
      tf2::fromMsg(pose, pose_tf);

      tf2::Transform offset_tf;
      tf2::Quaternion q;
      q.setRPY(0, 0, staging_yaw_offset_);
      offset_tf.setRotation(q);
      offset_tf.setOrigin(tf2::Vector3(staging_x_offset_, 0.0, 0.0));

      tf2::Transform staging_tf = pose_tf * offset_tf;
      tf2::toMsg(staging_tf, staging_pose.pose);

      staging_pose_pub_->publish(staging_pose);

      return staging_pose;
    }

    bool ChargerPresenceChargingDock::getRefinedPose(geometry_msgs::msg::PoseStamped& pose)
    {
      dock_pose_pub_->publish(pose);
      return true;
    }

    bool ChargerPresenceChargingDock::isDocked()
    {
      return is_charging_;
    }

    bool ChargerPresenceChargingDock::isCharging()
    {
      return is_charging_;
    }

    bool ChargerPresenceChargingDock::disableCharging()
    {
      return true;
    }

    bool ChargerPresenceChargingDock::hasStoppedCharging()
    {
      return !is_charging_;
    }

    }  // namespace open_mower_next::docking_helper

    PLUGINLIB_EXPORT_CLASS(open_mower_next::docking_helper::ChargerPresenceChargingDock, opennav_docking_core::ChargingDock)
    """,
}


def write_text(path: Path, content: str) -> None:
    normalized = textwrap.dedent(content).lstrip("\n")
    path.write_text(normalized, encoding="utf-8")


def main() -> int:
    if len(sys.argv) != 2:
        raise SystemExit("usage: apply-humble-adaptation.py <openmowernext-root>")

    root = Path(sys.argv[1]).resolve()
    if not root.exists():
        raise SystemExit(f"target path does not exist: {root}")

    for relative_path, marker in EXPECTED_MARKERS.items():
        path = root / relative_path
        text = path.read_text(encoding="utf-8")
        if marker not in text:
            raise SystemExit(f"unexpected upstream layout for {relative_path}")

    for relative_path, content in REWRITES.items():
        write_text(root / relative_path, content)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
