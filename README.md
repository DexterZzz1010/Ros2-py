# ROS2-Python

## 2. Basic concept

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

注意，文件名在launch文件和setup文件中使用

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

**定义package的名字和可执行文件。**

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

data_files中添加Launch文件的路径

entry_points中添加可执行文件的主函数

```python
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



### 修改setup.py 

data_files中添加Launch文件的路径

entry_points中添加可执行文件的主函数

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

```python
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



## 3. ROS Topic

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

读取该topic接收到的内容

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



The `.msg` files are composed of two parts: **fields** and **constants**. 字段和常数。



显示topic中包含的数据类型和数量

```
ros2 interface show <interface_name>
```

this command shows you a prototype interface so that you can use it.

```
ros2 interface proto geometry_msgs/msg/Twist
```

To publish a message in a particular Topic, use the following command:

```
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"
```

通过接口向topic中发送信息。Interface规范了topic中信息的类型？

The **`ros2 topic pub`** command is used to publish messages to a Topic. The command structure is as follows:

```
ros2 topic pub <topic_name> <interface_name> <message>
```

只执行一次：

```
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```



#### 4.Define a Topic

**Imagine a Topic as a pipe through which information flows, a channel where nodes can read or write information**.

### 3.2  Topic Publisher

#### 创建Simple Publisher Node

负责发送信息，控制turtlebot的角度和速度

#####  1.创建package：publisher_pkg

```
ros2 pkg create --build-type ament_python publisher_pkg --dependencies rclpy std_msgs geometry_msgs
```

#####  2.创建代码文件 **simple_publisher.py**

```python
import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist interface from the geometry_msgs package
from geometry_msgs.msg import Twist

class SimplePublisher(Node):

    def __init__(self):
        # Here you have the class constructor
        # call super() in the constructor to initialize the Node object
        # the parameter you pass is the node name
        super().__init__('simple_publisher')
        # create the publisher object
        # 创建发布者对象
        # in this case, the publisher will publish on /cmd_vel Topic with a queue size of 10 messages.
        # use the Twist module for /cmd_vel Topic
        # self.publisher_ = self.create_publisher(信息模型（接口）, '目标topic名', qu're'r)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # define the timer period for 0.5 seconds
        timer_period = 0.5
        # create a timer sending two parameters:
        # - the duration between 2 callbacks (0.5 seconds)
        # - the timer function (timer_callback)
        # 创建timer self.create_timer(时间间隔, 回调函数)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # 创建回调函数
        # Here you have the callback method
        # create a Twist message
       
        # 定义massage 类型为 twist
        msg = Twist()
        # define the linear x-axis velocity of /cmd_vel Topic parameter to 0.5
        msg.linear.x = 0.5
        # define the angular z-axis velocity of /cmd_vel Topic parameter to 0.5
        msg.angular.z = 0.5
        
        # Publish the message to the Topic
        # 发布信息
        self.publisher_.publish(msg)
        # Display the message on the console
        self.get_logger().info('Publishing: "%s"' % msg)
            
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    # 实例化
    simple_publisher = SimplePublisher()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(simple_publisher)
    # Explicity destroys the node
    simple_publisher.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

##### 3.创建launch文件

```
cd ~/ros2_ws/src/publisher_pkg
mkdir launch
cd ~/ros2_ws/src/publisher_pkg/launch
touch publisher_pkg_launch_file.launch.py
chmod +x publisher_pkg_launch_file.launch.py
```

##### 4.编译package

```
cd ~/ros2_ws
colcon build --packages-select publisher_pkg
source ~/ros2_ws/install/setup.bash
```

##### 5.Launch Node

```
ros2 launch publisher_pkg publisher_pkg_launch_file.launch.py
```



### 3.3  Topic Subscriber

#### 创建Simple Subscriber Node

负责接收信息，接收雷达信号

##### 创建**subscriber_pkg**

##### 创建**simple_subscriber.py**

```python
import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
# import Quality of Service library, to set the correct profile and reliability to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile


class SimpleSubscriber(Node):

    def __init__(self):
        # Here you have the class constructor
        # call super() in the constructor to initialize the Node object
        # the parameter you pass is the node name
        super().__init__('simple_subscriber')
        # create the subscriber object
        # in this case, the subscriptor will be subscribed on /scan topic with a queue size of 10 messages.
        # use the LaserScan module for /scan topic
        # send the received info to the listener_callback method.
        # 创建subscriber
        self.subscriber = self.create_subscription(
            LaserScan, # object
            '/scan',   # LaserScan module for /scan topic
            self.listener_callback, # 回调函数
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))  # is the most used to read LaserScan data and some sensor data.

    def listener_callback(self, msg):
        # print the log info in the terminal
        self.get_logger().info('I receive: "%s"' % str(msg))


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    # 实例化
    simple_subscriber = SimpleSubscriber()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(simple_subscriber)
    # Explicity destroy the node
    simple_subscriber.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- First, you define a callback method named **`listener_callback`** for the subscriber. This means that every time a message is published to the **`/scan`** topic, this method will be triggered.
- Second, you define a **`QoSProfile`**. This refers to the `Quality of Service` of the topic.



#####  创建Launch文件

```
mkdir launch
cd ~/ros2_ws/src/subscriber_pkg/launch
touch subscriber_pkg_launch_file.launch.py
chmod +x subscriber_pkg_launch_file.launch.py
```

Launch：

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='subscriber_pkg',
            executable='simple_subscriber',
            output='screen'),
    ])
```

创建setup.py文件

```python
from setuptools import setup
import os
from glob import glob

package_name = 'subscriber_pkg'

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
            'simple_subscriber = subscriber_pkg.simple_subscriber:main'
        ],
    },
)
```

compile

```
cd ~/ros2_ws
colcon build --packages-select subscriber_pkg
source ~/ros2_ws/install/setup.bash
```

运行launch文件

```
ros2 launch subscriber_pkg subscriber_pkg_launch_file.launch.py
```

#### QoS

```
ros2 topic echo -h
```

Publisher 和  Subscriber 的 QoS 设置要兼容。

查看QoS

```
ros2 topic info /scan -v
```



### 3.4  Write a Publisher & Subscriber Node

publisher ： 使用**`Twist`** messages.

Subscriber ：接收**`LaserScan`** messages

功能：左转绕圈直到五米内有墙，直走到离墙50cm，停

- Turn left until you get a value lower than 5m (from the laser direction in front of the robot). Then, move straight.
- If the distance received is greater than 50cm, move in a straight line towards the wall (do not go too fast!).
- If the distance received is less than 50cm, the robot stops.

#### 1.创建**exercise31_pkg**

```
ros2 pkg create --build-type ament_python exercise31_pkg --dependencies rclpy std_msgs sensor_msgs geometry_msgs
```

#### 2.创建****exercise31.py****

注意，文件名**exercise31.py**在launch文件和setup文件中使用

对于回调函数，构造并使用timer。

```python
import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

class Exercise31(Node):

    def __init__(self):
        # Here you have the class constructor
        # call the class constructor
        super().__init__('exercise31')
        
        # 创建 publisher
        # create the publisher object
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 创建 subscriber
        # create the subscriber object
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        
        # define the timer period for 0.5 seconds
        self.timer_period = 0.5
        
        # define the variable to save the received info
        self.laser_forward = 0
        
        # create a Twist message
        # 重复调用motion
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.motion)

	# 创建 subscriber 调用 laser_callback
    def laser_callback(self,msg):
        # Save the frontal laser scan info at 0°
        # 储存雷达信息
        self.laser_forward = msg.ranges[359] 
        
   # 定义运动方法     
    def motion(self):
        # print the data
        self.get_logger().info('I receive: "%s"' % str(self.laser_forward))
        # Logic of move
        if self.laser_forward > 5:
            self.cmd.linear.x = 0.5
            self.cmd.angular.z = 0.5
        elif self.laser_forward < 5 and self.laser_forward >= 0.5:
            self.cmd.linear.x = 0.2
            self.cmd.angular.z = 0.0         
        else:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            
        # Publishing the cmd_vel values to a Topic
        self.publisher_.publish(self.cmd)


            
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    exercise31 = Exercise31()       
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(exercise31)
    # Explicity destroy the node
    exercise31.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 3. 创建 launch file

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='exercise31_pkg',
            executable='exercise31',
            output='screen'),
    ])
```

#### 4. 构造 **setup.py**

```python
from setuptools import setup
import os
from glob import glob

package_name = 'exercise31_pkg'

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
            'exercise31 = exercise31_pkg.exercise31:main'
        ],
    },
)
```



### 3.5  How to Create a Custom Interface

创建一个名为custom_interfaces的新软件包。但是，此软件包必须是一个CMAKE软件包。

当前，无法在纯Python软件包中生成自定义接口。

但是，您可以在CMAKE软件包中创建自定义接口，然后在Python节点中使用它。

#### 步骤：

To create a new interface, you have to follow the next steps:

1. Create a directory named **`msg`** inside your package
2. Inside this directory, create a file named **`name_of_your_message.msg`** (more - information below)
3. Modify the `CMakeLists.txt` file (more information below)
4. Modify`package.xml` file (more information below)
5. Compile and source
6. Use in your node

#### 1. 创建文件夹

```
cd ~/ros2_ws/src/custom_interfaces
mkdir msg
```



#### 2. 创建一个 topic message 文件（.msg）

Age.msg:

在这里定义了Age有三个feature：year month day.

```
int32 year
int32 month
int32 day
```



#### 3.修改**CMakeLists.txt**

##### `find_package()`

找到这个包所需要的所有依赖

 In `package.xml`, state them as **`build_depend`** and **`exec_depend`**.

```
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


```

##### `rosidl_generate_interfaces()`

包括要编译此软件包的所有消息（在MSG文件夹中）

```
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Age.msg"
)
```



```cmake
cmake_minimum_required(VERSION 3.8)
project(custom_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Age.msg"
)

ament_package()
```



#### 4.修改**package.xml** 

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format2.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>custom_interfaces</name> 
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="ubuntu@todo.todo">ubuntu</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>

  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>

</package>
```

#### 5. 编译

```terminal
cd ~/ros2_ws
colcon build --packages-select custom_interfaces
source install/setup.bash
```

注意，每次创建新的message之后，需要在工作空间目录下**`source install/setup.bash`**。



测试一下：

```
ros2 interface show custom_interfaces/msg/Age
```

```c++
int32 year
int32 month
int32 day
```

### 3.6  使用 Custom Interface

#### 1. 创建新package**example36_pkg**

注意 需要添加依赖 也就是之前创建的 Interface **custom_interfaces**

```
ros2 pkg create --build-type ament_python example36_pkg --dependencies rclpy std_msgs geometry_msgs custom_interfaces
```

创建 **example36.py**

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
# 在这里 Import interface 中的数据类型 Age
from custom_interfaces.msg import Age


class Example36(Node):

    def __init__(self):
        # Here you have the class constructor
        # call the class constructor
        super().__init__('example36')
        # create the publisher object
        self.publisher_ = self.create_publisher(Age, 'age', 10)
        # create an Age message
        self.age = Age()
        # define the timer period for 0.5 seconds
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        # create an Age message
        self.age.year = 2031
        self.age.month = 5
        self.age.day = 21
        # publish the Age message
        self.publisher_.publish(self.age)


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    example36 = Example36()
    rclpy.spin(example36)
    # Explicity destroy the node
    example36.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

如何使用了新创建的Interface：

首先，Import interface 中的数据类型 Age。

在这里定义了Age有三个feature：year month day.

```
from custom_interfaces.msg import Age
```

同样的创建了该数据类型

```
self.age = Age()
```

最后使用了Age

```
def timer_callback(self):
    # create an Age message
    self.age.year = 2031
    self.age.month = 5
    self.age.day = 21
    # publish the Age message
    self.publisher_.publish(self.age)
```

You use every part of your interface to create a date and publish it into the `/age` topic.

#### 2. 创建 launch file

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='example36_pkg',
            executable='example36',
            output='screen'),
    ])
```

#### 3. 修改setup.py

data_files中添加Launch文件的路径

entry_points中添加可执行文件的主函数

```python
from setuptools import setup
import os
from glob import glob

package_name = 'example36_pkg'

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
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'example36 = example36_pkg.example36:main'
        ],
    },
)
```



#### 4. 编译 package

```
cd ~/ros2_ws
colcon build --packages-select example36_pkg
source ~/ros2_ws/install/setup.bash
```

#### 5. Launch the service

```
ros2 launch example36_pkg example36.launch.py
source ~/ros2_ws/install/setup.bash
ros2 topic echo /age
```





## 4. Understanding ROS2 Services

**What will you learn with this unit?**

- Understand a Service
- Basic Service commands
- Understand a Service Client
- Understand a Service Server
- Custom interfaces



### 4.1  什么是 ROS2 的 Service ?

在ROS2中，Services（服务）和Topics（话题）都是节点之间进行通信的方式。为了更好地理解它们，可以将它们与已知的概念进行比较。Topics使用发布-订阅模型，而Services使用调用-响应模型。就像你可以订阅一个Topic以接收特定信息并持续更新一样，Services只有在被显式调用时才提供数据，而调用方需要等待响应。

通过以下示例更好地理解这一点。考虑一个人脸识别系统，最好的方法是使用一个Service。然后，你的ROS2程序将在需要获得人物姓名时调用该Service（向Service发送请求）。然后，Service将返回一个响应，提供该人物的姓名。

为什么在这种情况下最好使用Service呢？因为当机器人周围没有人时，你不需要不断运行人脸检测节点。这将是对资源的大量浪费。你只需要在机器人面前有人时运行它。

在使用Services时，你将有两个角色：客户端（Clients）和服务器（Server）。你可以有多个客户端使用相同的服务服务器，但每个服务只能有一个服务器。

客户端发送request，服务器发送response

总而言之，如果需要实时流数据的广播，可能更适合使用Topics，

但无法确保消息的传输是否成功到达，且topic是一直在发布的；

而如果需要一对一的请求和响应通信，可能更适合使用Services，

但可能影响系统的并发性，且Services只有在被显式调用时才提供数据。

### 4.2  Basic Service Commands

```
ros2 service list
```

In this section, you will work with the `/moving` and `/stop` Services inside the red boxes.

#### 调用服务（发送请求）service call

```
ros2 service call <service_name> <service_type> <value>
```

#### 查看 servic-type

那么我们怎么知道这个service的type呢？ 有没有command？

```
ros2 service type /moving
ros2 service type <service_name> 
```



```
ros2 interface show std_srvs/srv/Empty
```

 然而我们发现只有  ---  ；但是这条分界线是很重要的。因为它将消息的请求(request)部分与响应(response)部分分离开来。相当于分别定义了输入和输出。

Now, try the `ros2 service call` command.

```
ros2 service call /moving std_srvs/srv/Empty
```

然后我们观察到机器人开始moving了

In summary, the last command you executed does the following:

- Look for a Service named **`/moving`**
- Create a Client along with a request and send it to the corresponding Service
- Wait for the Server response and print it

怎么停下来？调用stop 

```
ros2 service call /stop std_srvs/srv/Empty
```



### 4.3  创建 Service Client

#### **1. 创建新的package**  client_pkg

```
cd ~/ros2_ws/src
ros2 pkg create client_pkg --build-type ament_python --dependencies rclpy std_srvs
```

#### 2. 创建service_client.py

```python
p'y# import the empty module from std_servs Service interface
from std_srvs.srv import Empty
# import the ROS2 Python client libraries
import rclpy
from rclpy.node import Node


class ClientAsync(Node):

    def __init__(self):
        # Here you have the class constructor

        # call the class constructor to initialize the node as service_client
        super().__init__('service_client')
        
        
        # 创建Client对象，与 moving（server）链接
        # create the Service Client object
        # defines the name and type of the Service Server you will work with.
        self.client = self.create_client(Empty, 'moving')
        
        # 确保Service启动和运行
        # checks once per second if a Service matching the type and name of the Client is available.
        while not self.client.wait_for_service(timeout_sec=1.0):
            # if it is not available, a message is displayed
            self.get_logger().info('service not available, waiting again...')
        
        # create an Empty request
        self.req = Empty.Request()
        

    def send_request(self):
        # 使用call_async（）方法将异步请求发送到Service Server
        # 查看是否server是否有response
        # future: indicates whether the call and response are finished 
        # send the request
        self.future = self.client.call_async(self.req)


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    client = ClientAsync()
    # run the send_request() method
    client.send_request()

    while rclpy.ok():
        # pause the program execution, waits for a request to kill the node (ctrl+c)
        rclpy.spin_once(client)
        if client.future.done():
            try:
                # checks the future for a response from the Service
                # while the system is running. 
                # If the Service has sent a response, the result will be written
                # to a log message.
                response = client.future.result()
            except Exception as e:
                # Display the message on the console
                client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                # Display the message on the console
                client.get_logger().info(
                    'the robot is moving' ) 
            break

    client.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

#### 3. 创建Launch.py 文件

```
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='client_pkg',
            executable='service_client',
            output='screen'),
    ])
```



#### 4. 修改setup.py

data_files  （launch文件路径）

 add the entry points(可执行文件主函数)

```python
from setuptools import setup
import os
from glob import glob

package_name = 'client_pkg'

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
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'service_client = client_pkg.service_client:main',
        ],
    },
)
```

#### 5. 编译package

```
colcon build --packages-select client_pkg
source ~/ros2_ws/install/setup.bash
```

#### 6. Launch service node

```
ros2 launch client_pkg service_client_launch_file.launch.py
```



#### 7.  Code Review

创建client，连接到server类型为moving

```python
self.client = self.create_client(Empty, 'moving')
```



查看是否server是否有response

```python
self.future = self.client.call_async(self.req)
```

使用call_async（）方法将异步请求发送到服务服务器。然后，将服务器中的响应存储在变量self.future中。这个未来变量也非常重要。提出请求后，服务器将立即返回将来，这表明呼叫和响应是否完成（但不包含响应本身的值）。

接下来看main loop

检查server是否完成了response

```python
if client.future.done():
```

获取response

```python
response = client.future.result()
```

In this case, since you are working with an **`Empty`** response, you do nothing else.

#### 8. 创建 stop_client.py

连接到stop server

```python
# import the Empty module from std_servs Service interface
from std_srvs.srv import Empty
# import the ROS2 Python client libraries
import rclpy
from rclpy.node import Node


class ClientAsync(Node):

    def __init__(self):
        # Here you have the class constructor

        # call the class constructor to initialize the node as stop_client
        super().__init__('stop_client')
        
        ############################连接到stop server###############################
        
        # create the Service Client object
        # defines the name and type of the Service Server you will work with.
        self.client = self.create_client(Empty, 'stop')
        
        ############################################################################
        
        # checks once per second if a Service matching the type and name of the Client is available.
        while not self.client.wait_for_service(timeout_sec=1.0):
            # if it is not available, a message is displayed
            self.get_logger().info('service not available, waiting again...')
        
        # create an Empty request
        self.req = Empty.Request()
        

    def send_request(self):
        
        # send the request
        self.future = self.client.call_async(self.req)


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    client = ClientAsync()
    # run the send_request() method
    client.send_request()

    while rclpy.ok():
        # pause the program execution, waits for a request to kill the node (ctrl+c)
        rclpy.spin_once(client)
        if client.future.done():
            try:
                # checks the future for a response from the Service
                # while the system is running. 
                # If the Service has sent a response, the result will be written
                # to a log message.
                response = client.future.result()
            except Exception as e:
                # Display the message on the console
                client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                # Display the message on the console
                client.get_logger().info(
                    'the robot is stopped' ) 
            break

    client.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```



#### 9. 创建Launch.py

```
cd ~/ros2_ws/src/client_pkg/launch
touch stop_client_launch_file.launch.py
chmod +x stop_client_launch_file.launch.py
```



将可执行文件设为stop_client

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='client_pkg',
            executable='stop_client', #将可执行文件设为stop_client
            output='screen'),
    ])
```



#### 10. 修改setup.py

entry_points中 添加stop_client的main函数

```python
from setuptools import setup
import os
from glob import glob

package_name = 'client_pkg'

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
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'service_client = client_pkg.service_client:main',
        'stop_client = client_pkg.stop_client:main'    
        ],
    },
)
```



#### 11. 编译package

```
colcon build --packages-select client_pkg
source ~/ros2_ws/install/setup.bash
```



#### 12. Launch stop_client

```
ros2 launch client_pkg stop_client_launch_file.launch.py
```



### 4.4 Synchronous vs. Asynchronous Service Clients in ROS2































































