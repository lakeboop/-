# -# YLC 自动驾驶导航系统

基于 ROS2 Humble 的阿克曼转向自主导航平台，集成 SLAM 建图、AMCL 定位、路径规划与动态避障功能。

## 系统架构

```
┌─────────────────────────────────────────────────────┐
│                   Tkinter 竞赛控制面板                  │
│              (QR 方向选择 / 启停控制 / 状态监控)          │
└──────────────────────┬──────────────────────────────┘
                       │
┌──────────────────────▼──────────────────────────────┐
│              Nav2 导航框架 (Behavior Tree)             │
│  ┌──────────┐  ┌──────────┐  ┌───────────────────┐  │
│  │ Global   │  │ Local    │  │ Ackermann         │  │
│  │ Planner  │──│ Planner  │──│ RPP Controller    │  │
│  │ (A*/Hyb) │  │ (DWA)    │  │ (Pure Pursuit)    │  │
│  └──────────┘  └──────────┘  └───────────────────┘  │
└──────────────────────┬──────────────────────────────┘
                       │
┌──────────────────────▼──────────────────────────────┐
│                    底盘驱动层                          │
│  ┌──────────┐  ┌──────────┐  ┌───────────────────┐  │
│  │ origincar │  │ LS Lidar │  │ BMI088 IMU        │  │
│  │ _base     │  │ Driver   │  │ Driver            │  │
│  └──────────┘  └──────────┘  └───────────────────┘  │
└─────────────────────────────────────────────────────┘
```

## 硬件配置

| 组件 | 型号 | 说明 |
|------|------|------|
| 主控 | RDK X3 (地平线) | ROS2 运算平台 |
| 底盘 | OriginCar 阿克曼底盘 | 前轮转向/后轮驱动 |
| 激光雷达 | LS Lidar (LSM10P/LSN10) | 360° 2D 扫描 |
| IMU | BMI088 | 6轴惯性测量 |
| 深度相机 | Aurora 930 | RGB-D 感知 |
| USB 相机 | 通用 USB 摄像头 | 视觉识别 (QR码) |

## 编译部署

```bash
# 工作空间路径: /userdata/dev_ws/
cd /userdata/dev_ws/

# 完整编译
colcon build --symlink-install

# 选择性编译
colcon build --packages-select ylc_nav ylc_main mod qr_decoder

# 环境加载
source install/setup.bash
```

## 功能模块

### 1. 底盘控制 (`origincar_base`)
- 阿克曼运动学 (前轮转向角度 → 角速度转换)
- 串口通信控制
- `/cmd_vel` 话题接入

### 2. 导航系统 (`ylc_nav`)

| 节点 | 功能 | 说明 |
|------|------|------|
| `start_ylc_nav.launch.py` | 主导航启动 | SLAM/AMCL 模式切换 |
| `pipeline_navigator` | 流水线导航 | Nav2 NavigateThroughPoses 批量航点执行 |
| `low_cost_follower` | 分段路径跟踪 | 航点分组 + ComputePathThroughPoses + FollowPath |
| `ackermann_rpp_controller` | 阿克曼控制器插件 | 继承 Nav2 RPP，加入动态前瞻、弯道减速、卡死倒车恢复 |

### 3. 局部规划 (`ylc_main`)

| 节点 | 功能 |
|------|------|
| `DWAPlanner_node` | 动态窗口法 (DWA) 局部避障 |
| `goal_sequence_publisher` | 目标序列发布，集成 A* 全局规划 |
| `line_pub` | 基于 A* 代价地图的连线发布 |

### 4. 视觉识别

| 节点 | 功能 |
|------|------|
| `qr_code_detection_node` | ZBar 二维码检测 (话题: `/QR`) |
| `integrated_node` | 云端 API 图像上传与结果解析 |

### 5. 工具节点

| 节点 | 功能 |
|------|------|
| `tf_listener_node` | TF map→base_link 转发为 PoseWithCovariance |
| `footprint_publisher` | 机器人轮廓发布 (RViz 可视化) |
| `goal_publisher` | 周期性目标位姿发布 |
| `waypoint_visualizer` | RViz 航点标注 (圆球/立方体/折线) |
| `odom_imu_yaw_node` | Odom/IMU 偏航角实时对比 |
| `qr_code_listener_node` | QR 信号监听，触发节点关闭 |
| `image_compressor` | 原始图像 → JPEG 压缩转发 |

## 启动运行

### 建图模式 (SLAM)

```bash
# Gmapping 建图
ros2 launch ylc_nav start_ylc_nav.launch.py slam_mode:=True

# 或独立启动 Cartographer/Gmapping
ros2 launch ylc_main cartographer.launch.py
```

### 导航模式 (AMCL)

```bash
# 带 Nav2 DWB + AMCL
ros2 launch ylc_nav start_ylc_nav.launch.py slam_mode:=False map:=/path/to/map.yaml

# 或精简版 (仅 planner + controller)
ros2 launch ylc_nav small_nav.launch.py map:=/path/to/map.yaml
```

### 竞赛模式

```bash
# 完整竞赛流程 (控制面板 + 流水线导航)
ros2 launch ylc_nav ylc_main.launch.py

# DWA 避障 + 目标序列
ros2 launch ylc_main dwa_launch.launch.py

# AMCL 定位 + TF 转发
ros2 launch ylc_main location.launch.py map:=/path/to/map.yaml
```

### USB 相机 + AI 识别

```bash
# USB 相机 → 编码 → YOLOv8 DNN 检测 → WebSocket 推流
ros2 launch ylc_main usb_websocket_display.launch.py
```

### 键盘遥操作

```bash
# TCP 远程控制 (端口 9000)
ros2 run ylc_nav cobridge_client.py

# 或者标准键盘控制
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 关键话题列表

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/cmd_vel` | Twist | 发布 | 速度指令 |
| `/scan` | LaserScan | 订阅 | 激光雷达扫描 |
| `/odom` / `/odom_combined` | Odometry | 订阅 | 里程计 |
| `/imu/data_raw` | Imu | 订阅 | IMU 数据 |
| `/goal` / `/goal_pose` | PoseStamped | 订阅/发布 | 目标位姿 |
| `/map` | OccupancyGrid | 订阅 | 占据栅格地图 |
| `/amcl_pose` | PoseWithCovariance | 订阅 | AMCL 定位 |
| `/tf` / `/tf_static` | TFMessage | 双向 | 坐标变换 |
| `/footprint` | PolygonStamped | 发布 | 机器人轮廓 |
| `/sign` / `/sign_mod` | String | 订阅/发布 | 语义信号 |
| `/QR` / `/qr_sign` | String | 订阅/发布 | QR 码数据 |
| `/sign4return` | Int32 | 双向 | 反馈信号 |
| `/rosout` | Log | 订阅 | 系统日志 |

## 参数配置

关键参数通过 `config.yaml` 或 launch 文件中的 `params_file` 指定：

```yaml
# 阿克曼底盘参数
vehicle_wheelbase: 0.17          # 轴距 (m)
vehicle_max_steer_angle: 0.41    # 最大转向角 (rad)

# 速度限制
desired_linear_vel: 1.0          # 直线速度 (m/s)
turn_linear_vel: 0.5             # 转弯速度 (m/s)
max_backward_vel: 0.4            # 最大倒车速度 (m/s)
min_rotate_speed: 0.2            # 最小旋转速度 (m/s)

# DWA 参数
sim_time: 3.0                    # 前向模拟时间 (s)
sim_timestep: 0.5                # 模拟步长 (s)
scan_range: 1.0                  # 障碍物检测范围 (m)
weight_obs: 1.0                  # 障碍物代价权重
weight_to_goal: 0.8              # 目标代价权重
weight_speed: 0.4                # 速度代价权重

# 阿克曼 RPP PID
kp_angle: 2.2                    # 角度 P 增益
kd_angle: 1.8                    # 角度 D 增益
rotate_angle_threshold: 0.25     # 原地旋转触发阈值 (rad)
```

## 目录结构

```
/userdata/dev_ws/src/origincar/
├── origincar_base/          # 底盘驱动 (官方)
├── origincar_bringup/       # 启动配置 (官方)
├── origincar_description/   # URDF 模型 (官方)
├── origincar_msg/           # 自定义消息 (官方)
├── ylc_nav/                 # 导航功能包 ★
│   ├── ylc_nav/             # Python 节点
│   ├── src/                 # C++ 控制器插件
│   ├── include/             # 头文件
│   ├── launch/              # 启动文件
│   ├── config/              # Nav2 参数配置
│   ├── maps/                # 栅格地图
│   └── behavior_trees/      # 行为树 XML
├── ylc_main/                # 主控功能包 ★
│   ├── ylc_main/            # Python 节点
│   ├── src/                 # C++ DWA 规划器
│   ├── include/             # 头文件
│   ├── launch/              # 启动文件
│   └── config/              # DWA & AMCL 参数
├── mod/                     # 云端 API 模块 ★
├── qr_decoder/              # ZBar QR 码检测 ★
├── qr_listener/             # QR 信号监听 ★
├── image_compressor/        # 图像压缩转发 ★
├── utils/                   # 工具节点 (古月居)
├── LSlidar/                 # 激光雷达驱动
├── imu_bmi088/              # IMU 驱动
├── rf2o_laser_odometry/     # 激光里程计
├── robot_slam/              # SLAM 算法
├── 3rdparty/                # 第三方库
├── nav2_packages/           # Nav2 扩展
└── TinyNav/                 # TinyNav 端到端导航

★ = 自定义模块
```

## 远程控制协议

TCP 端口 `9000`，消息格式：

| 前缀 | 功能 | 示例 |
|------|------|------|
| `CMD_VEL:l,a` | 速度指令 | `CMD_VEL:0.5,0.2` |
| `SIGN:data` | 信号传输 | `SIGN:start` |
| `MAP_PUB` | 触发地图发布 | `MAP_PUB` |

## 许可证

本项目自定义模块采用 Apache 2.0 许可证。第三方组件遵循各自的许可条款。

- OriginCar SDK: 厂商授权
- Nav2: Apache 2.0
- ZBar: LGPL 2.1
- OpenCV: Apache 2.0
- Eigen3: MPL 2.0
- 古月居 image_transport_node: Apache 2.0
