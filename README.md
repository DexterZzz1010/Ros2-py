# ROS2-Python

## 2.Basic concept

### 2.1什么是 Package



```
source /opt/ros/humble/setup.bash
```

```
ros2 launch <package_name> <launch_file>
ros2 run <package_name> <executable_file>
```



Every Python package will have the following structure of files and folders:

- **`package.xml`** - File containing meta-information about the package (maintainer of the package, dependencies, etc.).
- **`setup.py`** - File containing instructions for how to compile the package.
- **`setup.cfg`** - File that defines where the scripts will be installed.
- **`src`/\<package_name>** - This directory will always have the same name as your package. You will put all your Python scripts inside this folder. It already contains an empty `__init__.py` file by default.

Some packages might contain extra folders. For instance, the **`launch`** folder contains the package's launch files (you will read more about it later).



### 2.3  Create a Package

Usually, the **ROS2 workspace** directory is called **`ros2_ws`**.

```
source /opt/ros/humble/setup.bash
```

```
cd ~/ros2_ws/
pwd
```

#### 创建package

进入src目录

创建基于python的package

```terminal
cd src
ros2 pkg create --build-type ament_python my_package --dependencies rclpy
```

package name: my_package
destination directory: /home/user/ros2_ws

build type: ament_python

Note also that we are specifying `ament_python` as the `build type`. This indicates that we are creating a Python package.

```
ros2 pkg create --build-type ament_python <package_name> --dependencies <package_dependency_1> <package_dependency_2>
```

#### 构建package

```
cd ~/ros2_ws/
colcon build
source install/setup.bash
```

#### 查看package

```
ros2 pkg list
ros2 pkg list | grep my_package
```

```
ros2_ws/
    src/
        my_package/
            package.xml
            setup.py
            ...
        my_package_2/
            package.xml
            setup.py
            ...
        my_package_x/
            package.xml
            setup.py
            ...
```

### 2.4  Compile a Package

注意 会编译整个src目录

```
cd ~/ros2_ws
colcon build
source install/setup.bash
```

只编译一个包：

```
colcon build --packages-select <package_name>
```



### 2.5  What is a Launch File?

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_teleop',
            executable='teleop_keyboard',
            output='screen'),
    ])
```

import modules:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
```

Next, define a function that will return a `LaunchDescription` object.

```
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            output='screen'),
    ])
```

Within the `LaunchDescription` object, generate a node where you will provide the following parameters:

1. `package=`Package name: Name of the package that contains the code of the ROS2 program to execute

   包的名字

2. `executable=`Name of python executable: Name of the Python executable file that you want to execut添加可执行文件

1. `output=`Type of output: Through which channel you will print the output of the program



### First ROS2 Program

#### 1.创建simple.py

```python
import rclpy
# import the Node module from ROS2 Python library
from rclpy.node import Node

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # print a message to the terminal
    print("Moe yo Byakugan! Kore ga watashi no nindō yo ")
    # english translation: "Blaze Away, Byakugan! This is My Ninja Way!"
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main() #call the main function
```

#### 2.创建launch文件

在包目录下创建launch文件夹和launch文件：

```
cd ~/ros2_ws/src/my_package
mkdir launch
cd ~/ros2_ws/src/my_package/launch
touch my_package_launch_file.launch.py
chmod +x my_package_launch_file.launch.py
```

my_package_launch_file.launch.py

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='simple_node',
            output='screen'),
    ])
```

#### 3.在工作空间目录下的setup.py:

Modify the setup.py file to generate an executable from your Python file.

生成可执行文件

```
from setuptools import setup
import os
from glob import glob

package_name = 'my_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='somebody very awesome',
    maintainer_email='user@user.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_node = my_package.simple:main'
        ],
    },
)
```

添加launch文件路径：(os.path.join('share', package_name), glob('launch/*.launch.py'))

#### 4.编译工作空间

```
cd ~/ros2_ws
colcon build
source install/setup.bash
```



### 创建setup.py 

#### 1.entry point

这段代码的主要目的是从刚才创建的脚本生成一个可执行文件。要做到这一点，需要使用一个名为entry_points的字典。在其中，可以找到一个名为console scripts的数组。

```python
import os
from glob import glob
from setuptools import setup

package_name = 'my_package'


setup(
    
    #code
    ...
    #code
    
    entry_points={
            'console_scripts': [
                'simple_node = my_package.simple:main'
            ],
        },
    
    #code
    ...
    
)
```

通过这一行，添加了一个entry_points，是my_package包中的simple文件的main函数

```
'<executable_name> = <package_name>.<script_name>:main'
```



#### 2. Data files

为了使colon在编译过程中可以找到launch文件，需要添加文件的路径

```
import os
from glob import glob
from setuptools import setup

package_name = 'my_package'


setup(
    
    #code
    ...
    #code
    
    data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
            (os.path.join('share', package_name), glob('launch/*.launch.py'))
        ],
    
    #code
    ...
    #code
    
)
```

glob('launch/*.launch.py')：从**`my_package`**的 `launch/` 文件夹，安装使用luanch文件, 到`~/ros2_ws/install/my_package/share/my_package/`.



### Ros Nodes

查看nodes

```
ros2 node list
```



重写simple.py

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        # call super() in the constructor to initialize the Node object
        # the parameter we pass is the node name
        super().__init__('Byakugan')
        # create a timer sending two parameters:
        # - the duration between two callbacks (0.2 seconds)
        # - the timer function (timer_callback)
        self.create_timer(0.2, self.timer_callback)
        
    def timer_callback(self):
        # print a ROS2 log on the terminal with a great message!
        self.get_logger().info("Moe yo Byakugan! Kore ga watashi no nindō yo")

def main(args=None):
    # initialize the ROS2 communication
    rclpy.init(args=args)
    # declare the node constructor
    node = MyNode()
    # keeps the node alive, waits for a request to kill the node (ctrl+c)
    rclpy.spin(node)
    # shutdown the ROS2 communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

代码解读：

创建一个MyNode类，继承Node。

该类存在两个方法， **`__init__`** (构造函数) 和 **`timer_callback`**（回调函数）.

**`__init__`**方法中， 初始化节点，名字叫 **`Byakugan`**:

```
super().__init__('Byakugan')
```

创建一个对象,timer

```
self.create_timer(0.2, self.timer_callback)
```

在**`timer_callback`** 中，向节点的log发送信息

```
self.get_logger().info("Moe yo Byakugan! Kore ga watashi no nindō yo")
```

在main函数中实例化node：

```
node = MyNode()

rclpy.spin(node)
```

**`spin()`** will keep the node alive and running until someone shuts it down



查看节点信息：

```
ros2 node info <node_name>
```

```
/Byakugan
  Subscribers:

  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /Byakugan/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /Byakugan/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /Byakugan/get_parameters: rcl_interfaces/srv/GetParameters
    /Byakugan/list_parameters: rcl_interfaces/srv/ListParameters
    /Byakugan/set_parameters: rcl_interfaces/srv/SetParameters
    /Byakugan/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:
```

### Client Libraries

The ROS2 team currently maintains the following client libraries:

- **`rclcpp`**: ROS2 client library for **C++**.
- **`rclpy`**: ROS2 client library for **Python**.



## 3.ROS Topic

### 3.1Basic Topic commands

显示当前环境中所有的topics

```
ros2 topic list
```

显示特定的topic

```
ros2 topic list | grep cmd_vel
```



#### 1.info

详细信息

```
ros2 topic info /cmd_vel
```

Okay. Now, break down what you have got a little bit:

- **Type:** Refers to the ROS2 interface associated with the Topic with which you need to work with this Topic. （接口）
- **Publisher count:** Refers to the number of active Publishers connected to the Topic.
- **Subscription count:** Refers to the number of active Subscribers connected to the Topic.

```
ros2 topic info <topic_name>
```



#### 2.echo

读取发布的topic

```
ros2 topic echo <topic_name>
```



#### 3. Interface

```
ros2 interface list
```



 The interfaces are divided into the following groups:

- **Messages:** Can be found as `.msg` files. They are simple text files that describe the fields of a ROS message. You can use them to generate source code for messages.

- 使用它们为消息生成源代码。

  

- **Services:** Can be found as `.srv` files. They are composed of two parts: a request and a response. Both are message declarations.

- 它们由两个部分组成：请求和答复。两者都是消息声明。



- **Actions:** Can be found as `.action` files. They are composed of three parts: a goal, a result, and feedback. Each part contains a message declaration.
- 它们由三个部分组成：目标，结果和反馈。每个部分都包含消息声明。



The `.msg` files are composed of two parts: **fields** and **constants**. 
