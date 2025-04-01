# ROS2 Vision Arm Control

本项目实现了一个基于计算机视觉的机器人手臂控制系统，使用 YOLO (You Only Look Once) 算法进行目标检测。系统运行在 ROS2 (Robot Operating System 2) 环境中，支持机器人系统各组件之间的高效通信。

## 项目结构

```
ros2_vision_arm_control
├── src
│   ├── ros2_vision_arm_control
│   │   ├── __init__.py
│   │   ├── arm_control.py        # 控制机器人手臂的逻辑
│   │   ├── camera_node.py        # 处理摄像头输入的 ROS2 节点
│   │   ├── yolo_detector.py      # YOLO 算法的目标检测集成
│   │   └── utils
│   │       ├── __init__.py
│   │       ├── arm_lib.py        # 与手臂控制相关的函数
│   │       ├── datasets.py       # 数据集加载和处理函数
│   │       ├── general.py        # 通用工具函数
│   │       ├── torch_utils.py    # PyTorch 工具函数
│   │       └── plot.py           # 可视化函数
├── package.xml                   # ROS2 包配置文件
├── setup.py                      # 包的安装脚本
└── README.md                     # 项目文档
```

## 安装步骤

按照以下步骤设置项目环境：

1. **克隆仓库：**
   ```bash
   git clone <repository-url>
   cd ros2_vision_arm_control
   ```

2. **安装依赖：**
   确保已安装并配置好 ROS2 环境。然后安装 `setup.py` 中列出的 Python 依赖：
   ```bash
   pip install -r requirements.txt
   ```

3. **构建包：**
   使用 `colcon` 构建工具构建项目：
   ```bash
   colcon build
   ```

4. **加载环境：**
   构建完成后，加载安装脚本：
   ```bash
   source install/setup.bash
   ```

## 使用方法

1. **启动摄像头节点：**
   该节点从摄像头捕获图像并处理以进行目标检测。
   ```bash
   ros2 run ros2_vision_arm_control camera_node
   ```

2. **运行 YOLO 检测器：**
   YOLO 检测器处理图像并返回检测结果。
   ```bash
   ros2 run ros2_vision_arm_control yolo_detector
   ```

3. **控制机器人手臂：**
   使用手臂控制脚本，根据检测结果管理机器人手臂的运动。
   ```bash
   ros2 run ros2_vision_arm_control arm_control
   ```

## 文件说明

- **`arm_control.py`**: 包含机器人手臂的运动控制逻辑。
- **`camera_node.py`**: 处理摄像头输入的 ROS2 节点。
- **`yolo_detector.py`**: 集成 YOLO 算法，用于目标检测。
- **`utils/`**: 包含多个实用工具模块：
  - `arm_lib.py`: 提供与手臂控制相关的函数。
  - `datasets.py`: 数据集加载和处理工具。
  - `general.py`: 通用工具函数。
  - `torch_utils.py`: PyTorch 相关工具。
  - `plot.py`: 数据可视化工具。

## 贡献指南

欢迎贡献代码！如果有任何改进建议或发现了 Bug，请提交 Issue 或 Pull Request。（并没有完成）

## 许可证

本项目基于 MIT 许可证开源。详情请参阅 LICENSE 文件。