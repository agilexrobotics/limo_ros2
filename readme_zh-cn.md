# 真机运行
## 克隆仓库
```shell
# 创建本地工作目录
mkdir -p ~/agx_workspace
cd ~/agx_workspace
# 克隆项目
git clone https://github.com/agilexrobotics/limo_ros2.git src
# 中国大陆用户加速下载
# git clone https://ghproxy.com/https://github.com/agilexrobotics/limo_ros2.git src

mv ~/agx_workspace/src/.devcontainer ~/agx_workspace
```
如果您需要 CUDA 环境，请移步 https://github.com/dusty-nv/jetson-containers 根据指导构建环境

【推荐】使用 VS Code remote 插件 连接到 limo，打开 ~/agx_workspace 后在菜单中选择 `reopen in container`

或运行自动配置脚本
```shell
cd ~/agx_workspace/src
chmod +x setup.sh
./docker_setup.sh
```
然后按照提示进行操作

# 仿真：
下面的脚本一条一条运行， 注意不能在后台运行 

## 前期准备
```shell
# 创建本地工作目录
mkdir -p ~/agx_workspace
cd ~/agx_workspace
# 克隆项目
git clone https://github.com/agilexrobotics/limo_ros2.git src
# 中国大陆用户加速下载
# git clone https://ghproxy.com/https://github.com/agilexrobotics/limo_ros2.git src

# 安装必要支持库
apt-get update \
    && apt-get install -y --no-install-recommends \	
    libusb-1.0-0 \
    udev \
    apt-transport-https \
    ca-certificates \
    curl \
    swig \
    software-properties-common \
    python3-pip


# 安装雷达驱动
git clone https://ghproxy.com/https://github.com/YDLIDAR/YDLidar-SDK.git &&\
    mkdir -p YDLidar-SDK/build && \
    cd YDLidar-SDK/build &&\
    cmake ..&&\
    make &&\
    make install &&\
    cd .. &&\
    pip install . &&\
    cd .. && rm -r YDLidar-SDK 

# 编译功能包
cd ~/agx_workspace
catkin_make
source devel/setup.bash
```

## 导航

```shell
rviz2
## 启动底盘
ros2 launch limo_bringup limo_start.launch.py
sleep 2

## 启动导航
ros2 launch limo_bringup limo_navigation.launch.py
sleep 2

## 启动定位
ros2 launch limo_bringup limo_localization.launch.py
```

## 建图

```shell
rviz2
ros2 launch limo_bringup limo_start.launch.py
ros2 launch build_map_2d revo_build_map_2d.launch.py
#上面三条指令启动之后，用遥控器控制车子行走
```


## 键盘控制

```shell
ros2 launch limo_bringup limo_start.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 四轮差速模式  

rviz2中显示模型

```
ros2 launch limo_description display_models_diff.launch.py 
```

gazebo中显示模型

```
ros2 launch limo_description gazebo_models_diff.launch.py 
```

启动键盘控制节点控制

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 阿克曼模式

开发中





