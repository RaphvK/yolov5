# YOLOv5 - TC

TC wrapper for the camera object detection repository [YOLOv5](https://github.com/ultralytics/yolov5).

> You need support or want to use this code for commercial purposes? Contact us at [info@thinking-cars.de](mailto:info@thinking-cars.de)!

## Quick Start

1. Start the inference node and the Web API

```bash
docker run --rm -it --name yolov5_tc -p 5000:5000 raffivk/yolov5_tc:latest
```

2. Navigate your web browser to [http://127.0.0.1:5000](http://127.0.0.1:5000) and upload a test image or directly send an image via REST API:

```bash
# yolov5_tc/
curl -X POST -F "image=@./test/unicaragil-vehicles.jpg" --output ./test/result.jpg http://localhost:5000/yolov5/input_image
```

3. Inspect the resulting image in the browser or at [./test/result.jpg](./test/result.jpg).

## Python Nodes

| Package | Node | Description |
| --- | --- | --- |
| `yolov5_tc` | `inference_node` | Receives images and publishes object lists |
| `yolov5_tc` | `image_publisher` | Publishes a test image for testig purposes |
| `yolov5_tc` | `web_api` | Starts a Webserver that receives images through a REST API, sends them to the `inference_node` and returns the image with bounding boxes as jpg file |

### yolov5_tc/inference_node

#### Subscribed Topics

| Topic | Type | Description |
| --- | --- | --- |
| `~/input_image` | `sensor_msgs/msg/Image` | Input image to be used for object detection |

#### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `~/output_objects` | `perception_msgs/msg/ObjectList` | List of detected objects |
| `~/output_image` | `sensor_msgs/msg/Image` | Input image with bounding boxes of detected objects overlayed |

### yolov5_tc/web_api

#### Subscribed Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/inference_node/output_image` | `sensor_msgs/msg/Image` | Image with bounding boxes to be returned through the REST API |

#### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/inference_node/input_image` | `sensor_msgs/msg/Image` | Image to be used for object detection that was received through the REST API |

### yolov5_tc/image_publisher

#### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/inference_node/input_image` | `sensor_msgs/msg/Image` | Test image to be used for object detection |

## Usage of [docker-ros](https://github.com/ika-rwth-aachen/docker-ros) Images

### Available Images

| Tag | Description |
| --- | --- |
| `latest` | latest version |

### Default Command

```bash
ros2 launch yolov5_tc launch.py
```

### Launch Files

| Package | File | Path | Description |
| --- | --- | --- | --- |
| `yolov5_tc` | `launch.py` | `/docker-ros/ws/install/yolov5_tc/share/yolov5_tc/launch.py` | Starts inference node and test node if `test:=true` is appended |


## Official Documentation

- [https://github.com/ultralytics/yolov5](https://github.com/ultralytics/yolov5)
