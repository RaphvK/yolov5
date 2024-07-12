# YOLOv5 - TC

TC wrapper for the camera object detection repository [YOLOv5](https://github.com/ultralytics/yolov5).

- [YOLOv5 - TC](#)
  - [Python Node](#python-node)
    - [\<PACKAGE\>/\<NODE\>](#packagenode)
      - [Subscribed Topics](#subscribed-topics)
      - [Published Topics](#published-topics)
      - [Services](#services)
      - [Actions](#actions)
      - [Parameters](#parameters)
  - [Usage of docker-ros Images](#usage-of-docker-ros-images)
    - [Available Images](#available-images)
    - [Default Command](#default-command)
    - [Environment Variables](#environment-variables)
    - [Launch Files](#launch-files)
    - [Configuration Files](#configuration-files)
    - [Additional Remarks](#additional-remarks)
  - [Official Documentation](#official-documentation)


## Python Nodes

| Package | Node | Description |
| --- | --- | --- |
| `yolov5_tc` | `inference_node` | Receives images and publishes object lists |
| `yolov5_tc` | `image_publisher` | Publishes a test image for testig purposes |

### yolov5_tc/inference_node

#### Subscribed Topics

| Topic | Type | Description |
| --- | --- | --- |
| `~/input` | `sensor_msgs/msg/Image` | Input image |

#### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `~/output` | `perception_msgs/msg/ObjectList` | List of detected objects |

### yolov5_tc/image_publisher

#### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/inference_node/input` | `sensor_msgs/msg/Image` | Test image |

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
