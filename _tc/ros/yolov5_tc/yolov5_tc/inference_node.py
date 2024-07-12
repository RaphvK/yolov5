import rclpy
from sensor_msgs.msg import Image
from perception_msgs.msg import ObjectList
from rclpy.node import Node

# constants
INPUT_TOPIC = "~/input"
OUTPUT_TOPIC = "~/output"

class InferenceNode(Node):

    def __init__(self):

        super().__init__("inference_node")

        self.setup()

    def setup(self):
        """Set up subscribers, publishers, and more.
        """

        # create a subscriber for handling incoming messages
        self.subscriber = self.create_subscription(Image,
                                                   INPUT_TOPIC,
                                                   self.topicCallback,
                                                   qos_profile=1)
        self.get_logger().info(f"Subscribed to '{self.subscriber.topic_name}'")

        # create a publisher for publishing messages
        self.publisher = self.create_publisher(ObjectList,
                                               OUTPUT_TOPIC,
                                               qos_profile=1)
        self.get_logger().info(f"Publishing to '{self.publisher.topic_name}'")

    def topicCallback(self, msg: Image):

        # echo received message
        self.get_logger().info(f"Received image with stamp: '{msg.header.stamp}'")

        # skip if the published topic has no subscription
        # if self.publisher.get_subscription_count() == 0:
        #     return

        # TODO Inference


        # create and publish a new message
        object_list = ObjectList()
        self.get_logger().info(f"Publishing Object List with {len(object_list.objects)} objects")
        self.publisher.publish(object_list)

def main():

    rclpy.init()
    rclpy.spin(InferenceNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
