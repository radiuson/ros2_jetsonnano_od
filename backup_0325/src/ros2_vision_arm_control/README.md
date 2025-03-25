# ROS2 Vision Arm Control

This project implements a robotic arm control system using computer vision techniques, specifically leveraging the YOLO (You Only Look Once) algorithm for object detection. The system is designed to operate within a ROS2 (Robot Operating System 2) environment, facilitating communication between various components of the robotic system.

## Project Structure

```
ros2_vision_arm_control
├── src
│   ├── ros2_vision_arm_control
│   │   ├── __init__.py
│   │   ├── arm_control.py        # Logic for controlling the robotic arm
│   │   ├── camera_node.py        # ROS2 node for handling camera input
│   │   ├── yolo_detector.py      # Integration of YOLO for object detection
│   │   └── utils
│   │       ├── __init__.py
│   │       ├── arm_lib.py        # Functions related to arm control
│   │       ├── datasets.py       # Dataset loading and processing functions
│   │       ├── general.py        # General utility functions
│   │       ├── torch_utils.py    # PyTorch utility functions
│   │       └── plot.py           # Visualization functions
├── package.xml                   # ROS2 package configuration
├── setup.py                      # Package setup script
└── README.md                     # Project documentation
```

## Installation

To set up the project, follow these steps:

1. **Clone the repository:**
   ```
   git clone <repository-url>
   cd ros2_vision_arm_control
   ```

2. **Install dependencies:**
   Ensure you have ROS2 installed and set up. Then, install any additional Python dependencies listed in `setup.py`.

3. **Build the package:**
   ```
   colcon build
   ```

4. **Source the setup script:**
   ```
   source install/setup.bash
   ```

## Usage

1. **Launch the camera node:**
   This node captures images from the camera and processes them for object detection.
   ```
   ros2 run ros2_vision_arm_control camera_node
   ```

2. **Run the YOLO detector:**
   The YOLO detector processes images and returns detection results.
   ```
   ros2 run ros2_vision_arm_control yolo_detector
   ```

3. **Control the robotic arm:**
   Use the arm control script to manage the movements of the robotic arm based on detection results.
   ```
   ros2 run ros2_vision_arm_control arm_control
   ```

## Contributing

Contributions are welcome! Please open an issue or submit a pull request for any improvements or bug fixes.

## License

This project is licensed under the MIT License. See the LICENSE file for details.