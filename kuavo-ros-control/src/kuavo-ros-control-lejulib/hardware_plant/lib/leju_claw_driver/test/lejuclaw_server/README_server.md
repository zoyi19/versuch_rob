# LEJU 夹爪控制服务器使用说明

包含两个组件：
- **lejuclaw_server** - C++服务器程序
- **lejuclaw_controller.py** - Python客户端脚本

通过JSON文件进行通信。

## 使用方法

### 1. 编译

在ROS工作空间根目录下执行：

```bash
catkin build hardware_node
```

### 2. 启动夹爪控制（服务端）

```bash
# 在终端1中运行
./devel/lib/hardware_node/lejuclaw_server
```

服务器启动后会初始化夹爪并运动到50%位置，然后等待控制指令。

### 3. 发送控制指令（客户端）

```bash
# 在终端2中运行（进入lejuclaw_server目录）
cd src/kuavo-ros-control-lejulib/hardware_plant/lib/leju_claw_driver/test/lejuclaw_server
```

命令格式：`python3 lejuclaw_controller.py <左夹爪百分比位置> <右夹爪百分比位置>`。
0为开爪，100为关爪。

```bash
python3 lejuclaw_controller.py 50.0 50.0    # 左右都运动到50%位置
python3 lejuclaw_controller.py 0.0 0.0      # 左右都运动到0%位置
python3 lejuclaw_controller.py 100.0 100.0  # 左右都运动到100%位置
python3 lejuclaw_controller.py 80.0 20.0    # 左80%，右20%
```

## 退出

按 `Ctrl+C` 退出，夹爪会运动到50%位置、清零电流并关闭系统。

## 注意事项

- 确保只有一个服务器实例在运行
- 命令文件会被服务器自动删除
- 服务器每100ms检查一次命令文件
- 位置值范围：0-100（百分比）
