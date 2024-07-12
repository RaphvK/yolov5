import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from flask import Flask, request, jsonify, Response
import time
import cv2
import numpy

app = Flask(__name__)
node = None
bridge = CvBridge()

class WebApi(Node):
    def __init__(self):
        super().__init__('web_api')
        self.publisher_ = self.create_publisher(Image, '/inference_node/input_image', 10)
        self.subscription = self.create_subscription(
            Image,
            '/inference_node/output_image',
            self.image_callback,
            10
        )
        self.output_image = None

    def image_callback(self, msg):
        self.get_logger().info('Received output image')
        self.output_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')


@app.route('/yolov5/input_image', methods=['POST'])
def receive_image():

    file_bytes = numpy.fromstring(request.files['image'].read(), numpy.uint8)
    image_cv = cv2.imdecode(file_bytes, cv2.IMREAD_UNCHANGED)
    image_msg = bridge.cv2_to_imgmsg(image_cv, encoding='bgr8')
    node.get_logger().info('Received image via REST API')
    node.publisher_.publish(image_msg)

    # wait for inference
    t0 = time.time()
    while node.output_image is None:
        if time.time() - t0 > 10:
            return jsonify({'error': 'Inference took too long'}), 500
        time.sleep(0.1)
        rclpy.spin_once(node)
    output_image = node.output_image
    node.output_image = None
    # Convert output image to JPEG format
    _, output_image_jpg = cv2.imencode('.jpg', output_image)

    # Convert JPEG image to bytes
    output_image_bytes = output_image_jpg.tobytes()

    # Return output image as JPEG file
    return Response(output_image_bytes, mimetype='image/jpeg')

def main(args=None):
    rclpy.init(args=args)
    global node
    node = WebApi()
    app.run(host='0.0.0.0', port=5000)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()