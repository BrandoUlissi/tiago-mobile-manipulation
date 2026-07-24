import subprocess
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class state_machine_1_node(Node):
    def __init__(self):
        super().__init__('state_machine_1')
        self.subscription = self.create_subscription(
            String,
            'example_topic',
            self.topic_callback,
            10)
        self.subscription  # prevent unused variable warning

    def topic_callback(self, msg):
        self.get_logger().info(f'Received message: {msg.data}')

def execute_command(command, blocking):
    if blocking:
        result = subprocess.run(command, capture_output=True, text=True)
        if result.returncode != 0:
            print(f"Error running {' '.join(command)}: {result.stderr}")
        else:
            print(f"Successfully ran {' '.join(command)}")
        return result.returncode
    else:
        process = subprocess.Popen(command)
        print(f"Started {' '.join(command)} in non-blocking mode")
        return process

def main(args=None):
    rclpy.init(args=args)
    state_machine_1 = state_machine_1_node()

    commands = [
        (['ros2', 'launch', 'tiago_gazebo', 'tiago_gazebo.launch.py', 'group_number:=38'], False),
        (['ros2', 'launch', 'tiago_2dnav', 'tiago_nav_bringup.launch.py', 'is_public_sim:=false', 'rviz:=True', 'slam:=True'], False),
        (['ros2', 'launch', 'tiago_safe_position', 'safe_position_launch.py'], False),
        (['ros2', 'launch', 'explore_lite', 'explore.launch.py'], False),
        (['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', 'map'], True)
    ]

    for command, blocking in commands:
        execute_command(command, blocking)
        if not blocking:
            time.sleep(20)

    rclpy.spin(state_machine_1)
    state_machine_1.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()