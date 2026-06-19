from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    detection_node = Node(
        package='detection',
        executable='detection',
        name='detection',
        output='screen',
    )

    # The camera driver publishes with frame_id 'default_cam' but the URDF defines
    # it as 'gripper_camera_link', breaking the TF chain map→...→wrist→default_cam.
    # This manually links default_cam to wrist using the real offset from the URDF,
    # since the upstream fix was pushed too late to safely update the robot.
    
    # The gripper_camera_link has just recently been added: https://github.com/mirte-robot/mirte-ros-packages/blob/main/mirte_description/mirte_master_description/urdf/arm.xacro
    # It does not exist yet with our version, so we will manually create a static transform based on the values in the arm.xacro file
    
    # We basically manually append the tf default_cam to the wrist
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.027',
            '--y', '0',
            '--z', '-0.067',
            '--roll', '1.5707963267949',
            '--pitch', '0',
            '--yaw', '1.5707963267949',
            '--frame-id', 'wrist',
            '--child-frame-id', 'default_cam'
        ]
    )

    ld = LaunchDescription()
    ld.add_action(static_tf)
    ld.add_action(detection_node)
    return ld