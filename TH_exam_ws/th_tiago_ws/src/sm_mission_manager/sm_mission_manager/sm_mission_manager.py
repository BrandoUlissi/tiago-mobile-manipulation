import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from std_msgs.msg import Empty, Bool
import time
import os
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class StateMachineNode(Node):

    def __init__(self):
        super().__init__('state_machine_node')

        # State variables
        self.state = 0
        self.exploration_start_time = None  # Variabile per memorizzare il tempo di inizio esplorazione
        
        # QoS Profiles
        qos_profile_high = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.start_mission_subscriber = self.create_subscription(
            Empty, '/start_mission', self.start_mission_callback, qos_profile_sensor_data)

        self.switch_on_slam_subscriber = self.create_subscription(
            Empty, '/arm_reached_safe_pos', self.switch_on_slam_callback, qos_profile_sensor_data)
        
        self.obj_detected_subscriber = self.create_subscription(
            Bool, '/truck_tracking_status', self.obj_detected_callback, qos_profile_sensor_data)

        # Publishers
        self.move_arm_in_safe_pos_publisher = self.create_publisher(
            Empty, '/move_arm_in_safe_pos', qos_profile_high)

        self.explore_resume_publisher = self.create_publisher(
            Bool, '/explore/resume', qos_profile_high)

        self.head_joint_trajectory_publisher = self.create_publisher(
            JointTrajectory, '/head_controller/joint_trajectory', qos_profile_high)

        self.target_follow_publisher = self.create_publisher(
            Empty, '/start_target_following', qos_profile_high)


        # Flags
        self.is_start_mission_msg_received = False
        self.is_arm_in_safe_pos = False
        self.is_exploration_started = False
        self.exploration_ended = False
        self.object_detected = False
        self.is_head_tilted = False  # Aggiungi questa variabile

        self.timer_period = 0.01
        self.state_machine_timer = self.create_timer(self.timer_period, self.run_state_machine)

    # Callbacks
    def start_mission_callback(self, msg):
        self.is_start_mission_msg_received = True

    def switch_on_slam_callback(self, msg):
        self.is_arm_in_safe_pos = True
    
    def obj_detected_callback(self, msg):
        if msg.data:
            self.object_detected = True

    def on_head_tilt_complete(self):
        self.get_logger().info("[STATE 1]: Head tilt completed, transitioning to state 2")
        self.state = 2  # Passa allo stato 2
        self.head_tilt_timer.cancel()  # Cancella il timer
        
    def run_state_machine(self):
        if self.state == 0:
            self.get_logger().info("[STATE 0]: Waiting for the /start_mission message")
            if self.is_start_mission_msg_received:
                self.state = 1
                self.get_logger().info("[STATE 0]: /start_mission message received")
        
        elif self.state == 1:
            if not self.is_head_tilted:
                self.get_logger().info("[STATE 1]: Publishing head joint trajectory")
                
                # Crea il messaggio JointTrajectory
                joint_trajectory_msg = JointTrajectory()
                joint_trajectory_msg.joint_names = ['head_1_joint', 'head_2_joint']
                point = JointTrajectoryPoint()
                point.positions = [0.0, -0.35]
                point.time_from_start.sec = 2
                point.time_from_start.nanosec = 0
                joint_trajectory_msg.points.append(point)
                
                # Pubblica il messaggio
                self.head_joint_trajectory_publisher.publish(joint_trajectory_msg)
                self.get_logger().info(f"[STATE 1]: Published head joint trajectory: {joint_trajectory_msg}")
                
                # Imposta un timer per attendere il completamento del movimento
                self.head_tilt_timer = self.create_timer(2.0, self.on_head_tilt_complete)  # Attendi 2 secondi
                self.is_head_tilted = True  # Impedisci la ripubblicazione del messaggio
            else:
                self.get_logger().info("[STATE 1]: Waiting for head tilt to complete...")

        elif self.state == 2:
            self.get_logger().info("[STATE 2]: Moving the arm to a safe position")
            self.move_arm_in_safe_pos_publisher.publish(Empty())
            if self.is_arm_in_safe_pos:
                self.state = 3
                self.exploration_start_time = time.time()  # Memorizza il tempo di inizio esplorazione
                self.get_logger().info("[STATE 2]: Arm in safe position")

        elif self.state == 3:
            #self.get_logger().info("[STATE 3]: EXPLORATION PHASE")
            # Publish the resume exploration message
            if not self.is_exploration_started and not self.exploration_ended:
                resume_msg = Bool()
                resume_msg.data = True
                self.explore_resume_publisher.publish(resume_msg)
                self.is_exploration_started = True

            #self.get_logger().info(f"[STATE 3]: THE ELAPSED TIME IS {time.time() - self.exploration_start_time}")
            
            if self.object_detected:
                #self.get_logger().info("[STATE 3]: Object detected, transitioning to state 4")
                self.exploration_ended = True
                resume_msg = Bool()
                resume_msg.data = False
                self.explore_resume_publisher.publish(resume_msg)
                self.state = 4
            
        elif self.state == 4:
            self.target_follow_publisher.publish(Empty())
            #self.get_logger().info("[STATE 4]: Published start target following message")
            
            

# Main entry point
def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    # Begin state machine
    node.run_state_machine()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()