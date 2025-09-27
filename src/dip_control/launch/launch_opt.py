import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument  # Import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration  # Import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch DipSimulator with LQRController for simulation, loading parameters from a YAML file specified
    as a launch argument, with a default file path.
    """

    # --- Begin Argument Declaration ---

    # 1. Construct the full path to the *default* YAML parameters file
    default_params_file = os.path.join(
        get_package_share_directory("dip_control"),
        "config",
        "lqr_opt_params.yaml",  # Ensure this file exists in your package
    )

    # 2. Declare launch arguments
    # This allows users to override the parameters file at runtime (not required in this project)
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Path to the YAML parameters file for LQR controller",
    )

    # --- End Argument Declaration ---

    # --- Node Definition ---

    # 3. Use LaunchConfiguration to reference the declared argument
    # This will be the default path unless overridden by the user
    resolved_params_file = LaunchConfiguration("params_file")

    # --- Nodes ---

    # lqr_controller Node
    lqr_controller = Node(
        package="dip_control",
        executable="lqr_opt",
        name="lqr_controller_opt",
        output="screen",
        emulate_tty=True,
        # Pass the LaunchConfiguration object (which resolves to the path)
        # inside a list to the parameters argument. The launch system handles
        # substituting the actual path string here.
        parameters=[resolved_params_file],  # Path to your parameters file
    )

    # dip_simulator Node
    dip_simulator = Node(
        package="dip_sim",
        executable="simulate_opt",
        name="dip_simulator_opt",
        output="screen",
        emulate_tty=True,
    )

    # --- Create the launch description ---
    ld = LaunchDescription()
    ld.add_action(params_file_arg)
    ld.add_action(dip_simulator)
    ld.add_action(lqr_controller)

    return ld
