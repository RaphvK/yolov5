import rclpy
import itertools
from sensor_msgs.msg import Image
from perception_msgs.msg import ObjectList, Object, ObjectClassification
from rclpy.node import Node
from cv_bridge import CvBridge
import torch

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

        self.cv_bridge = CvBridge()
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

        self.class_mapping = {
            0: 1, # person
            1: 2, # bicycle
            2: 4, # car
            3: 3, # motorcycle
            4: 0, # airplane
            5: 7, # bus
            6: 10, # train
            7: 5, # truck
            8: 0, # boat
            9: 9, # traffic light --> use different state model!
            10: 9, # fire hydrant
            11: 9, # stop sign
            12: 9, # parking meter
            13: 9, # bench
            14: 8, # bird
            15: 8, # cat
            16: 8, # dog
            17: 8, # horse
            18: 8, # sheep
            19: 8, # cow
            20: 8, # elephant
            21: 8, # bear
            22: 8, # zebra
            23: 8, # giraffe
            24: 0, # backpack
            25: 0, # umbrella
            26: 0, # handbag
            27: 0, # tie
            28: 0, # suitcase
            29: 0, # frisbee
            30: 0, # skis
            31: 0, # snowboard
            32: 0, # sports ball
            33: 0, # kite
            34: 0, # baseball bat
            35: 0, # baseball glove
            36: 0, # skateboard
            37: 0, # surfboard
            38: 0, # tennis racket
            39: 0, # bottle
            40: 0, # wine glass
            41: 0, # cup
            42: 0, # fork
            43: 0, # knife
            44: 0, # spoon
            45: 0, # bowl
            46: 0, # banana
            47: 0, # apple
            48: 0, # sandwich
            49: 0, # orange
            50: 0, # broccoli
            51: 0, # carrot
            52: 0, # hot dog
            53: 0, # pizza
            54: 0, # donut
            55: 0, # cake
            56: 0, # chair
            57: 0, # couch
            58: 0, # potted plant
            59: 0, # bed
            60: 0, # dining table
            61: 0, # toilet
            62: 0, # TV
            63: 0, # laptop
            64: 0, # mouse
            65: 0, # remote
            66: 0, # keyboard
            67: 0, # cell phone
            68: 0, # microwave
            69: 0, # oven
            70: 0, # toaster
            71: 0, # sink
            72: 0, # refrigerator
            73: 0, # book
            74: 0, # clock
            75: 0, # vase
            76: 0, # scissors
            77: 0, # teddy bear
            78: 0, # hair drier
            79: 0, # toothbrush
        }

    def topicCallback(self, msg: Image):

        # echo received message
        self.get_logger().info(f"Received image with stamp: '{msg.header.stamp}'")

        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        result = self.model(cv_image)
        result_pandas = result.pandas().xyxy[0]

        # create and publish a new message
        object_list = ObjectList()
        object_list.header.stamp = msg.header.stamp
        object_list.header.frame_id = msg.header.frame_id

        for (xmin, xmax, ymin, ymax, confidence, class_id) in zip(result_pandas['xmin'], result_pandas['xmax'], result_pandas['ymin'], result_pandas['ymax'], result_pandas['confidence'], result_pandas['class']):

            common_class_id = self.class_mapping[class_id]

            if common_class_id != 0:

                print(xmin, xmax, ymin, ymax, confidence, class_id)

                object_classification = ObjectClassification()
                object_classification.type = common_class_id

                object = Object()
                object.existence_probability = confidence
                object.state.header.stamp = msg.header.stamp
                object.state.header.frame_id = msg.header.frame_id
                object.state.model_id = 16
                object.state.continuous_state = [xmin, ymin, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, xmax-xmin, ymax-ymin]
                object.state.classifications = [object_classification]

                object_list.objects.append(object)            

        self.get_logger().info(f"Publishing Object List with {len(object_list.objects)} objects")
        self.publisher.publish(object_list)

def main():

    rclpy.init()
    rclpy.spin(InferenceNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
