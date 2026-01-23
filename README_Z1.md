# 宇树Z1机械臂集成指南

## 概述

本指南详细说明如何将原有的xArm6机器人控制系统替换为宇树Z1机械臂控制系统，实现完全相同的抓取功能。

## 系统架构

```
原有系统: YOLO检测 → xArm6控制 → 抓取执行
新系统:   YOLO检测 → Z1机械臂控制 → 抓取执行
```

## 安装步骤

### 1. 硬件连接

1. **网络配置**
   - 将PC与Z1机械臂通过网线连接
   - 配置PC IP地址为: `192.168.123.x` (x为1-254之间的任意值)
   - Z1机械臂默认IP: `192.168.123.110`

2. **连接测试**
   ```bash
   ping 192.168.123.110
   ```

### 2. 软件依赖安装

#### 2.1 安装Z1 SDK

```bash
# 克隆Z1控制器仓库
cd ~
mkdir -p z1_ws/src
cd z1_ws/src
git clone https://github.com/unitreerobotics/z1_controller.git

# 安装依赖
sudo apt install -y libboost-dev libeigen3-dev
sudo ln -s /usr/include/eigen3/Eigen /usr/local/include/Eigen
sudo ln -s /usr/include/eigen3/unsupported /usr/local/include/unsupported

# 安装pybind11 (用于Python接口)
git clone https://github.com/pybind/pybind11.git
cd pybind11
mkdir build && cd build
cmake .. -DPYBIND11_TEST=OFF
make -j
sudo make install
```

#### 2.2 编译Z1 SDK

```bash
cd ~/z1_ws/src/z1_controller
mkdir build && cd build
cmake ..
make -j

# 编译Python接口
cd ../z1_sdk
mkdir build && cd build
cmake ..
make -j
```

### 3. 项目配置

#### 3.1 文件结构

```
robot_grasp/
├── core/
│   ├── wrap_z1.py          # Z1机械臂控制封装类
│   ├── wrap_xarm.py        # 原xArm6控制类(保留)
│   ├── grasp.py            # 主抓取程序(已修改)
│   └── ...
├── test_z1_arm.py          # Z1机械臂测试脚本
└── README_Z1.md           # 本文档
```

#### 3.2 关键文件说明

**wrap_z1.py**: Z1机械臂控制封装类
- 兼容原xArm6接口设计
- 提供位置控制、夹爪控制、位姿读取等功能
- 包含运动边界限制和错误处理

**grasp.py**: 主抓取程序(已修改)
- 将`xarm6()`替换为`z1_arm()`
- 保持其他功能不变

## 使用方法

### 1. 启动Z1控制器

```bash
# 终端1: 启动Z1控制器
cd ~/z1_ws/src/z1_controller/build
./z1_ctrl
```

### 2. 测试Z1机械臂

```bash
# 终端2: 运行测试脚本
cd /path/to/robot_grasp
python3 test_z1_arm.py
```

### 3. 运行抓取程序

```bash
# 终端2: 运行主抓取程序
cd /path/to/robot_grasp/core
python3 grasp.py --date test
```

## API接口说明

### z1_arm类主要方法

```python
# 初始化
robot = z1_arm(ip='192.168.123.110')

# 移动控制
robot.move([x, y, z])              # 移动到指定位置
robot.move_init_pose()             # 移动到初始位姿
robot.rand_action()                # 执行随机动作

# 夹爪控制
robot.open_gripper()               # 打开夹爪
robot.close_gripper()              # 关闭夹爪

# 抓取序列
robot.grasp([x, y, z])             # 完整抓取动作

# 状态查询
pose = robot.get_current_pose()    # 获取当前位姿
```

## 工作空间参数

### Z1机械臂工作范围

```python
# 运动范围限制 (单位: 米)
limit_xyz = [
    [0.1, 0.6],    # X轴范围
    [-0.3, 0.3],   # Y轴范围  
    [0.0, 0.5]     # Z轴范围
]
```

### 初始位姿

```python
# 初始位姿 (x, y, z, roll, pitch, yaw)
init_pose = [0.3, 0.0, 0.2, 0, 0, 0]
```

## 故障排除

### 1. 连接问题

**问题**: 无法连接到Z1机械臂
```
解决方案:
- 检查网线连接
- 确认IP地址配置正确
- 使用ping命令测试连通性
```

### 2. SDK导入问题

**问题**: `ImportError: No module named 'unitree_arm_interface'`
```
解决方案:
- 确认Z1 SDK已正确编译
- 检查Python路径配置
- 重新编译Python接口
```

### 3. 运动控制问题

**问题**: 机械臂无响应或运动异常
```
解决方案:
- 确认Z1控制器正在运行
- 检查机械臂是否处于使能状态
- 查看错误日志信息
```

## 性能对比

| 特性 | xArm6 | Z1机械臂 |
|------|-------|----------|
| 自由度 | 6 | 6 |
| 工作半径 | ~500mm | ~600mm |
| 负载能力 | ~1kg | ~2kg |
| 控制精度 | ±0.1mm | ±0.1mm |
| 通信接口 | TCP/IP | TCP/IP |

## 注意事项

1. **安全操作**
   - 上电前确保机械臂处于零位
   - 运行前检查工作空间内无障碍物
   - 紧急情况下按急停按钮

2. **坐标系统**
   - Z1使用右手坐标系
   - 与xArm6的坐标系可能存在差异
   - 需要重新进行手眼标定

3. **运动限制**
   - 避免超出机械臂工作范围
   - 注意关节角度限制
   - 防止机械臂碰撞

## 开发建议

1. **渐进式替换**
   - 先在仿真环境中测试
   - 逐步替换控制功能
   - 保留原xArm6代码作为备份

2. **参数调优**
   - 根据实际应用调整运动速度
   - 优化抓取轨迹
   - 调整夹爪参数

3. **扩展功能**
   - 添加力控制
   - 实现更复杂的抓取策略
   - 集成视觉伺服

## 技术支持

- **宇树官方文档**: https://dev-z1.unitree.com/
- **GitHub仓库**: https://github.com/unitreerobotics/z1_controller
- **技术社区**: Unitree开发者论坛

## 版本历史

- v1.0: 初始Z1机械臂集成
- v1.1: 添加测试脚本和错误处理
- v1.2: 优化API接口兼容性