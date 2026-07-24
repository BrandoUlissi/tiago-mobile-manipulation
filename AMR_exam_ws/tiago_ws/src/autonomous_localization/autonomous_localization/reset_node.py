import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
"""
This script defines a ROS2 node that asynchronously calls the 'reinitialize_global_localization' service.
It uses the std_srvs/Empty service type to trigger global localization reset in a robot system.
"""

class MinimalClientAsync(Node):
    """
    MinimalClientAsync is a ROS2 node that acts as a client to the 'reinitialize_global_localization' service.
    It waits for the service to become available and then sends an asynchronous request.
    """
    def __init__(self):
        super().__init__('reinitialize_global_localization_client')  # Initialize the node with a name
        self.cli = self.create_client(Empty, 'reinitialize_global_localization')  # Create a client for the Empty service
        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.req = Empty.Request()  # Create an empty request object

    def send_request(self):
        """
        Send an asynchronous request to the service.
        The result will be stored in self.future.
        """
        self.future = self.cli.call_async(self.req)


def main():
    """
    Main function to initialize the ROS2 Python client, send the service request,
    and handle the response.
    """
    rclpy.init()  # Initialize ROS2 Python client library
    minimal_client = MinimalClientAsync()  # Create the client node
    minimal_client.send_request()  # Send the service request
    # Spin until the future is done (response received)
    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()  # Get the result of the service call
            except Exception as e:
                minimal_client.get_logger().info('Call failed: %r' % (e,))  # Log failure
            else:
                minimal_client.get_logger().info('Call succeeded!')  # Log success
                break
    minimal_client.destroy_node()  # Clean up the node
    rclpy.shutdown()  # Shutdown ROS2

if __name__ == '__main__':
    # Entry point of the script
    main()

