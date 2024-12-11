import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from threading import Thread

class InputPublisher(Node):
    def __init__(self):
        super().__init__('chat_input')
        self.publisher = self.create_publisher(String, "/llm_input_audio_to_text", 10)
        self.subscriber = self.create_subscription(String, "/llm_feedback_to_user", self.listener_callback, 10)

    def publish_string(self, string_to_send):
        msg = String()
        msg.data = string_to_send
        self.publisher.publish(msg)  # Use publisher here

    def listener_callback(self, msg):
        print(f"AI: {msg.data}")


def input_thread(input_publisher):
    while True:
        user_input = input("You: ")
        if user_input == "exit":
            break
        input_publisher.publish_string(user_input)


def main(args=None):
    rclpy.init(args=args)
    input_publisher = InputPublisher()

    # Create a separate thread to handle user input
    thread = Thread(target=input_thread, args=(input_publisher,))
    thread.start()

    # Spin the node to keep ROS 2 communications active
    rclpy.spin(input_publisher)

    # Wait for the input thread to finish
    thread.join()

    # Clean up
    input_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
