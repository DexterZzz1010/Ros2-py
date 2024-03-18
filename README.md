# ROS2-Python

## 1 Workspace

### **工作空间是什么**

类似的，在ROS机器人开发中，我们针对机器人某些功能进行代码开始时，各种编写的代码、参数、脚本等文件，也需要放置在某一个文件夹里进行管理，这个文件夹在ROS系统中就叫做**工作空间**。

所以工作空间是一个存放项目开发相关文件的文件夹，也是**开发过程中存放所有资料的大本营**。

ROS系统中一个典型的工作空间结构如图所示，这个dev_ws就是工作空间的根目录，里边会有四个子目录，或者叫做四个子空间。

- **src，代码空间**，未来编写的代码、脚本，都需要人为的放置到这里；
- **build，编译空间**，保存编译过程中产生的中间文件；
- **install，安装空间**，放置编译得到的可执行文件和脚本；
- **log，日志空间**，编译和运行过程中，保存各种警告、错误、信息等日志。

总体来讲，这四个空间的文件夹，我们绝大部分操作都是在src中进行的，编译成功后，就会执行install里边的结果，build和log两个文件夹用的很少。

### **创建工作空间**

了解了工作空间的概念和结果，接下来我们可以使用如下命令创建一个工作空间，并且下载教程的代码：

```
$ mkdir -p ~/dev_ws/src
$ cd ~/dev_ws/src
$ git clone https://gitee.com/guyuehome/ros2_21_tutorials.git
```

### **自动安装依赖**

我们从社区中下载的各种代码，多少都会有一些依赖，我们可以手动一个一个安装，也可以使用rosdep工具自动安装：

```
解释$ sudo apt install -y python3-pip
$ sudo pip3 install rosdepc
$ sudo rosdepc init
$ rosdepc update
$ cd ..
$ rosdepc install -i --from-path src --rosdistro humble -y
```

### **编译工作空间**

依赖安装完成后，就可以使用如下命令编译工作空间啦，如果有缺少的依赖，或者代码有错误，编译过程中会有报错，否则编译过程应该不会出现任何错误：

```
$ sudo apt install python3-colcon-ros
$ cd ~/dev_ws/
$ colcon build
```

### **设置环境变量**

编译成功后，为了让系统能够找到我们的功能包和可执行文件，还需要设置环境变量：

```
$ source install/local_setup.sh # 仅在当前终端生效
$ echo " source ~/dev_ws/install/local_setup.sh" >> ~/.bashrc # 所有终端均生效
```

至此，我们就完成了工作空间的创建、编译和配置

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



### 2.6 colcon_cd

You can also confirm that your package is there by using the **colcon_cd** command. This command, however, is not enabled by default.

To enable this feature, you have to source the shell script `colcon_cd.sh`, provided by the **colcon_cd** package.

```shell
cd /home/user
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=/home/user/ros2_ws" >> ~/.bashrc
. .bashrc
```

You can now use the command like this:

```
colcon_cd my_package
```

The command should now take you to the path where you have the package **`my_package`**: `/home/user/ros2_ws/src/my_package`.

If you use the command without specifying any package name, it will take you to root of your ROS2 workspace: `/home/user/ros2_ws`.

```shell
colcon_cd
```

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

**注意，launch中的executable应当与setup.py-entry_points-console_scripts的名字相同。**

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



#### 3. Interface ( .msg / .srv / .action )

```
ros2 interface list
```

Interface分为三种 Messages， Services， Actions

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
        # self.publisher_ = self.create_publisher(信息模型（接口）, '目标topic名', queue size )
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
            LaserScan, # module
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



### 3.5  How to Create a Custom Interface (.msg)

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

**也就是自定义发布的信息的类型和内容。**

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

**客户端发送request，服务器发送response**

总而言之，如果需要实时流数据的广播，可能更适合使用Topics，

但无法确保消息的传输是否成功到达，且topic是一直在发布的；

而如果需要一对一的请求和响应通信，可能更适合使用Services，

但可能影响系统的并发性，且Services只有在被显式调用时才提供数据。

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_ROS2/images/ros2-services.gif)

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

#### 查看interface

```
ros2 interface show std_srvs/srv/Empty
```

 然而我们发现只有  ---  ；但是这条分界线是很重要的。因为它将消息的请求(request)部分与响应(response)部分分离开来。相当于分别定义了输入和输出。

#### ros2 service call

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

**客户端发送request，服务器发送response**

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
    # 发送请求
    
    
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

同步实例：

```python
# initialize the ROS communication
rclpy.init(args=args)
# declare the node constructor
client = ClientSync()
# start the communication thread
spin_thread = Thread(target=rclpy.spin, args=(client,))
spin_thread.start()
# run the send_request() method
response = client.send_request()
# Display the message on the console
client.get_logger().info('Pretty message') 

minimal_client.destroy_node()
# shutdown the ROS communication
rclpy.shutdown()
```

异步一般来说是绝对安全的，同步可能会带来死锁的问题。

如果您使用过ROS1，您可能记得服务是同步的。现在使用ROS2，服务默认情况下是异步的。这并不意味着您不能拥有同步服务，但不建议这样做。



### 4.5  Create a Service Server

Server 接收request，发布response

**一个Server要包括以下几个内容：**

**type, name, and callback function**，回调函数包括publisher



**任何request 都要通过回调函数接收**

**任何response 都要通过回调函数发布**

**在回调函数中，不仅可以接收和回复response，还可以publish message**



/moving 这个server的代码：

```python
# import the Empty module from std_servs Service interface
from std_srvs.srv import Empty
# import the Twist module from geometry_msgs messages interface
from geometry_msgs.msg import Twist
# import the ROS2 Python client libraries
import rclpy
from rclpy.node import Node


class Service(Node):

    def __init__(self):
        # Here you have the class constructor

        # call the class constructor to initialize the node as service_moving
        super().__init__('service_moving')
        
        # 创建Server对象 type, name, and callback function
        # create the Service Server object
        # defines the type, name, and callback function
        self.srv = self.create_service(Empty, 'moving', self.empty_callback)
        # create the Publisher object
        # in this case, the Publisher will publish on /cmd_vel topic with a queue size of 10 messages.
        
        # 在publisher 定义发布信息的类型
        # use the Twist module
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        

    def empty_callback(self, request, response):
        # The callback function receives the self-class parameter, 
        # received along with two parameters called request and response
        # - receive the data by request
        # - return a result as a response
		
        # create a Twist message
        # 根据发布信息的类型，定义msg的内容
        msg = Twist()
        # define the linear x-axis velocity of /cmd_vel topic parameter to 0.3
        msg.linear.x = 0.3
        # define the angular z-axis velocity of /cmd_vel topic parameter to 0.3
        msg.angular.z = 0.3
        # Publish the message to the Topic
        # 发布信息
        self.publisher_.publish(msg)
        # print a pretty message
        self.get_logger().info('RUN ROBOT RUN!')
        
        
        # return the response parameter
        return response


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    moving_service = Service()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(moving_service)
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- The Service uses an **`Empty`** type
- The name of the Service is **`moving`**
- The Service callback is **`empty_callback`**

/stop 的代码：

基本和/moving一样，msg内容和server名字不同。

```python
# import the empty module from std_servs Service interface
from std_srvs.srv import Empty
# import the Twist module from geometry_msgs messages interface
from geometry_msgs.msg import Twist
# import the ROS2 Python client libraries
import rclpy
from rclpy.node import Node


class Service(Node):

    def __init__(self):
        # Here you have the class constructor

        # call the class constructor to initialize the node as service_stop
        super().__init__('service_stop')
        # create the Service server object
        # defines the type, name, and callback function
        self.srv = self.create_service(Empty, 'stop', self.empty_callback)
        # create the Publisher object
        # in this case, the Publisher will publish on /cmd_vel topic with a queue size of 10 messages.
        # use the Twist module
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        

    def empty_callback(self, request, response):
        # The callback function receives the self-class parameter, 
        # received along with two parameters called request and response
        # - receive the data by request
        # - return a result as a response

        # create a Twist message
        msg = Twist()
        # define the linear x-axis velocity of /cmd_vel topic parameter to 0
        msg.linear.x = 0.0
        # define the angular z-axis velocity of /cmd_vel topic parameter to 0
        msg.angular.z = 0.0
        # Publish the message to the topic
        self.publisher_.publish(msg)
        # print a pretty message
        self.get_logger().info('Stop there, Robot!')
        
        # return the response parameter
        return response


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor  
    service = Service()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(service)
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```



Based on the previous examples, make a Service that executes the following instructions:

- Create a new package and call it **`exercise42_pkg`** with `rclpy`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, and `std_srvs` as dependencies.
- Use the previous Service Server codes to create a new Service Server that makes the robot turn right (as in the example of the **`/moving`** Service, but the opposite direction). But this time will be a little bit more difficult. You will use the Service **`SetBool`**, part of the **`std_srv`** package. It will work like this:
  - When the request is **`true`**, the robot turns right.
  - When the input is **`false`**, the robot stops!
- Create the launch file to start your node.
- Test your Service by calling it using the **`ros2 service call`** command.



#### 1. 创建Package exercise42_pkg

```
ros2 pkg create exercise42_pkg --build-type ament_python --dependencies rclpy std_msgs geometry_msgs sensor_msgs std_srvs
```

#### 2. 创建exercise42.py

Server type（Interface的类型）：SetBool

```
bool data # e.g. for hardware enabling / disabling
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages
```

SetBool ：	接收bool 类型的request （True/False）

​						发送bool和string类型的response

Server名字：moving_right

回调函数：self.SetBool_callback  

**在回调函数中，不仅可以接收和回复response，还可以publish message**

```python
# import the SetBool module from std_servs Service interface
from std_srvs.srv import SetBool
# import the Twist module from geometry_msgs messages interface
from geometry_msgs.msg import Twist
# import the ROS2 Python client libraries
import rclpy
from rclpy.node import Node


class Service(Node):

    def __init__(self):
        # Here you have the class constructor

        # call the class constructor to initialize the node as service_moving
        super().__init__('service_moving_right')
        
        
        #######################创建Server#################################
        
        # create the Service Server object
        # defines the type, name, and callback function
        
        # type：SetBool
		# Server名字：moving_right
		# 回调函数：self.SetBool_callback 
        self.srv = self.create_service(SetBool, 'moving_right', 	self.SetBool_callback)
       
    	#################################################################
    
    	########################创建Publisher#############################
    	
        # create the Publisher object
        # in this case, the Publisher will publish on /cmd_vel topic with a queue size of 10 messages.
        # use the Twist module
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # create a Twist message
        self.cmd = Twist()    
        #################################################################

    def SetBool_callback(self, request, response):
        # The callback function receives the self-class parameter, 
        # received along with two parameters called request and response
        # - receive the data by request
        # - return a result as a response        
            
        # Publish the message to the topic
        # As you see, the name of the request parameter is data, so do it
        if request.data == True:
            
            # define the linear x-axis velocity of /cmd_vel topic parameter to 0.3
            self.cmd.linear.x = 0.3
            # define the angular z-axis velocity of /cmd_vel topic parameter to 0.3
            self.cmd.angular.z =-0.3
            
            # 发布message
            self.publisher_.publish(self.cmd)
            # You need a response
            response.success = True
            # You need another response, but this time, SetBool lets you put a String
            response.message = 'MOVING TO THE RIGHT RIGHT RIGHT!'

        if request.data == False:

            self.cmd.linear.x = 0.0
            # define the angular z-axis velocity of /cmd_vel topic parameter to 0.3
            self.cmd.angular.z =0.0
            
            self.publisher_.publish(self.cmd)
            response.success = False

            response.message = 'It is time to stop!'       
                
        # return the response parameters
        return response


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    moving_right_service = Service()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(moving_right_service)
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```



#### 3. 创建Launch文件

```
cd ~/ros2_ws/src/exercise42_pkg
mkdir launch
cd ~/ros2_ws/src/exercise42_pkg/launch
touch exercise42_launch_file.launch.py
chmod +x exercise42_launch_file.launch.py
```

**exercise42_launch_file.launch.py**

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='exercise42_pkg',
            executable='exercise42',
            output='screen'),
    ])
```



#### 4. 修改setup.py

```python
from setuptools import setup
import os
from glob import glob

package_name = 'exercise42_pkg'

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
            'exercise42 = exercise42_pkg.exercise42:main'
        ],
    },
)
```

#### 5. 编译**package**

```
colcon build --packages-select exercise42_pkg
source ~/ros2_ws/install/setup.bash
```

#### 6. **Launch the Service Server node in your Terminal #1.**

```
ros2 launch exercise42_pkg exercise42_launch_file.launch.py
```

#### 7.使用ros2 service call in your Terminal #2

调用服务 控制机器人

```
ros2 service call /moving_right std_srvs/srv/SetBool data:\ true
ros2 service call /moving_right std_srvs/srv/SetBool data:\ false
```



### 4.6  Custom Service Interface (.srv)

**自定义Server Type**

To create a new Service type (srv), complete the following steps:

1. Create a directory named **`srv`** inside your package
2. Inside this directory, create a file named **`Name_of_your_service_type.srv`** (more - information below)
3. Modify **`CMakeLists.txt`** file (more information below)
4. Modify **`package.xml`** file (more information below)
5. Compile and source
6. Use in code

For example, create an interface that receives as a request three possible movements: `"Turn Right"` (turn right) , `"Turn Left"` (turn left) and `"Stop"` (stop).

#### 1. 创建目录

```
cd ~/ros2_ws/src/custom_interfaces
mkdir srv
cd srv
touch MyCustomServiceMessage.srv
```

#### 2.MyCustomServiceMessage.srv  

包含request和response：

request： 接收一个字符串变量`move`, 有三种情况，分别为"Turn right" "Turn left" "Stop"

response：返回一个bool值 ，查看是否接收到并成功execute。

```
string move   # Signal to define the movement
              # "Turn right" to make the robot turn in the right direction.
              # "Turn left" to make the robot turn in the left direction. 
              # "Stop" to make the robot stop the movement.

---
bool success  # Did it achieve it?
```

#### 3. **CMakeLists.txt**

包括两个函数

 `find_package()`

这是声明编译topic、service和action的消息所需的所有包。

```
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
```

 `rosidl_generate_interfaces()`

此函数包括要编译的该软件包的所有接口（也就是所需的 .msg / .srv / .action文件）。

```
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MyCustomServiceMessage.srv"
)
```

In your case, you will also have the message interfaces here:

```
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Age.msg"
  "srv/MyCustomServiceMessage.srv"
)
```

CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.5)
project(custom_interfaces)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter, which checks for copyrights
  # uncomment the line when copyright and license are not present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MyCustomServiceMessage.srv"
)

ament_package()
```

#### 4. 修改 package.xml

添加以下依赖

```
<build_depend>rosidl_default_generators</build_depend>

<exec_depend>rosidl_default_runtime</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

This is the minimum expression of the `package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>custom_interfaces</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="user@todo.todo">user</maintainer>
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

```
cd ~/ros2_ws
colcon build --packages-select custom_interfaces
source install/setup.bash
```

#### 6. 查看结果 

```
ros2 interface show custom_interfaces/srv/MyCustomServiceMessage
```

### 4.7 使用 Custom Interface

#### 1. 创建 Package **`movement_pkg`**

```cmd
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python movement_pkg --dependencies rclpy custom_interfaces std_msgs geometry_msgs sensor_msgs
```

#### 2.创建两个新的可执行文件

  **movement_server.py**

```python
# import the Twist module from geometry_msgs messages interface
from geometry_msgs.msg import Twist
# import the MyCustomServiceMessage module from custom_interfaces package
from custom_interfaces.srv import MyCustomServiceMessage
# import the ROS2 Python client libraries
import rclpy
from rclpy.node import Node

class Service(Node):

    def __init__(self):
        # Here you have the class constructor

        # call the class constructor to initialize the node as service_stop
        super().__init__('movement_server')
        
        
        # create the Service Server object
        # defines the type, name, and callback function
        self.srv = self.create_service(MyCustomServiceMessage, 'movement', self.custom_service_callback)
        
        
        # create the Publisher object
        # in this case, the Publisher will publish on /cmd_vel topic with a queue size of 10 messages.
        # use the Twist module
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        

    def custom_service_callback(self, request, response):
        # The callback function receives the self-class parameter, 
        # received along with two parameters called request and response
        # - receive the data by request
        # - return a result as a response

        # create a Twist message
        msg = Twist()
        ####################################################
        
        # 根据不同的string move ，发布内容不同的Twist message
        
        if request.move == "Turn Right":
            # define the linear x-axis velocity of /cmd_vel topic parameter to 0.1
            msg.linear.x = 0.1
            # define the angular z-axis velocity of /cmd_vel topic parameter to -0.5 to turn right
            msg.angular.z = -0.5
            # Publish the message to the topic
            self.publisher_.publish(msg)
            # print a pretty message
            self.get_logger().info('Turning to right direction!!')
            # response state
            response.success = True
        elif request.move == "Turn Left":
            # define the linear x-axis velocity of /cmd_vel topic parameter to 0.1
            msg.linear.x = 0.1
            # define the angular z-axis velocity of /cmd_vel topic parameter to 0.5 to turn left
            msg.angular.z = 0.5
            # Publish the message to the topic
            self.publisher_.publish(msg)
            # print a pretty message
            self.get_logger().info('Turning to left direction!!')
            # response state
            response.success = True
        elif request.move == "Stop":
            # define the linear x-axis velocity of /cmd_vel topic parameter to 0
            msg.linear.x = 0.0
            # define the angular z-axis velocity of /cmd_vel topic parameter to 0
            msg.angular.z = 0.0
            # Publish the message to the topic
            self.publisher_.publish(msg)
            # print a pretty message
            self.get_logger().info('Stop there!!')
            # response state
            response.success = True
        else:
            # response state
            response.success = False
        
        # return the response parameter
        return response


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor  
    service = Service()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(service)
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**movement_client.py**

```python
# import the MyCustomServiceMessage module from custom_interfaces package
from custom_interfaces.srv import MyCustomServiceMessage
# import the ROS2 Python client libraries
import rclpy
from rclpy.node import Node
import sys


class ClientAsync(Node):

    def __init__(self):
        # Here you have the class constructor

        # call the class constructor to initialize the node as movement_client
        super().__init__('movement_client')
        
        # create the Service Client object
        # defines the name and type of the Service Server you will work with.
        self.client = self.create_client(MyCustomServiceMessage, 'movement')
        # checks once per second if a Service matching the type and name of the client is available.
        while not self.client.wait_for_service(timeout_sec=1.0):
            # if it is not available, a message is displayed
            self.get_logger().info('service not available, waiting again...')
        
        # create an Empty request
        self.req = MyCustomServiceMessage.Request()
        

    def send_request(self):
        
        ###################################################################
        # send the request
        # 在此处定义，由命令行输入的 string move 
        self.req.move = sys.argv[1]
        ###################################################################
        
        # uses sys.argv to access command line input arguments for the request.
        self.future = self.client.call_async(self.req)
        # to print in the console


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
                # if the Service has sent a response, the result will be written
                # to a log message.
                response = client.future.result()
            except Exception as e:
                # Display the message on the console
                client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                # Display the message on the console
                client.get_logger().info(
                    'Response state %r' % (response.success,))
            break

    client.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```



#### 3. 创建**launch file**

**`movement_server_launch_file.launch.py`** to launch the Service Server node you just created.

```
cd ~/ros2_ws/src/movement_pkg
mkdir launch
cd launch
touch movement_server_launch_file.launch.py
chmod +x movement_server_launch_file.launch.py
```

**`movement_server_launch_file.launch.py`**

只添加server的可执行文件

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='movement_pkg',
            executable='movement_server',
            output='screen'),
    ])
```

#### 4. 修改 setup.py

install the launch file you have just created and add the entry points to the executable scripts.

```python
from setuptools import setup
import os
from glob import glob

package_name = 'movement_pkg'

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
        'movement_server = movement_pkg.movement_server:main',
        'movement_client = movement_pkg.movement_client:main'
        ],
    },
)
```

#### 5. 编译

```
cd ~/ros2_ws
colcon build --packages-select movement_pkg
source ~/ros2_ws/install/setup.bash
```

#### 6. **Launch the Service Server**

```
ros2 launch movement_pkg movement_server_launch_file.launch.py
```



#### 7. **move the robot**

```
ros2 run <package_name> <executable_file>
```

```
ros2 run movement_pkg movement_client "Turn Left"
ros2 run movement_pkg movement_client "Turn Right"
ros2 run movement_pkg movement_client "Stop"
```



#### 8. code review

```
self.req.move = sys.argv[1]
```

This will capture the first argument used in the command and store it in **`self.req.move`**, which as you already know, is the request part of the Service type.

So, if you use a command like the below one:

```
ros2 run movement_pkg movement_client "Stop"
```

Here, **`"Stop"`** is the argument, and it will be stored as a string in **`self.req.move`**.



## Service总结

#### 1. Call service的两种方式：

在命令行`ros2 service call`+server名+特定的server interface(server类型)

创建**client**，使用`ros2 run` 或者 `ros2 launch`调用client去Call service

```python
        ##################### 创建Client对象，与 moving（server）链接################
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
    
    #################################### 发送请求 ############################ 
    client.send_request()
```



Service 接收到request后呢，可能会publish一些message到别的nodes，然后response返回client。

#### 2. 创建 server

Server 接收request，发布response

**一个Server要包括以下几个内容：**

**type, name, and callback function**，回调函数包括publisher



**任何request 都要通过回调函数接收**

**任何response 都要通过回调函数发布**

**在回调函数中，不仅可以接收和回复response，还可以publish message

```python
#######################创建Server#################################
        
        # create the Service Server object
        # defines the type, name, and callback function
        
        # server type：SetBool
		# Server名字：moving_right
		# 回调函数：self.SetBool_callback 
        
        # server中包括一些publiser
        
        self.srv = self.create_service(SetBool, 'moving_right', 	self.SetBool_callback)
       
#################################################################
```

#### 3. 创建 client

```python
################## 创建Client对象，与 moving（server）链接#############

        # create the Service Client object
        # defines the name and type of the Service Server you will work with.
        # Service Server type: Empty
        # Service Server name : moving
        self.client = self.create_client(Empty, 'moving')
        
####################################################################
```

#### 4. 自定义Interface（Service Server type） ( .srv )

创建一个package ,分别创建三个文件夹 `srv`,`msg`,`action`

在srv目录下创建一个.srv文件MyCustomServiceMessage：

```
string move   # Signal to define the movement
              # "Turn right" to make the robot turn in the right direction.
              # "Turn left" to make the robot turn in the left direction. 
              # "Stop" to make the robot stop the movement.

---
bool success  # Did it achieve it?
```

修改Cmakelists.txt和package.xml。编译。

使用的时候记得import

```
from custom_interfaces.srv import MyCustomServiceMessage
```





## 5.Callbacks

当我们创建一个基于Publisher的Ros2程序时，就要使用到Callback

回调函数是一种通过将函数作为参数传递给其他函数，使其在特定事件发生时执行的编程模式。

**所谓的回调函数（Callbacks），通俗来说就是由事件出发的函数。**

### 5.2 Setup

- **创建新的包：** 为了容纳本单元生成的所有代码，首先创建一个新的ROS 2包。
- **引入`ament_cmake`：** 引入 `ament_cmake` 作为新的角色。在ROS 2中，可以使用 `ament_cmake` 或 `ament_python` 来创建ROS 2包。
- **选择 `ament_cmake`：** 如果使用 `Python` 脚本，可以将其放置在由 `ament_cmake` 或 `ament_python` 构建的ROS 2包中。在这个单元中，选择了 `ament_cmake`，因为它在使用 `Python` 脚本时更加清晰和易于使用。
- **`ament_cmake`的作用：** `ament_cmake` 用于配置ROS 2包的编译方式，以及查找包内的所有文件。它规定了ROS 2 colcon构建系统如何安装文件。
- **简洁和易用：** 在这个单元中选择了 `ament_cmake`，因为在经验中它更加简洁和易于使用，特别是在处理 `Python` 脚本时。
- **包内文件：** 在创建包的同时，还会创建用于编写代码的所有文件，以便能够专注于这些文件中的实际概念。
- **后续课程中的 `ament_python`：** 在本课程的其余部分使用 `ament_python`，以充分发挥 `Python` 的全部功能。



#### 1. 创建Package basics_ros2_multithreading

```cmd
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake basics_ros2_multithreading --dependencies rclpy

mkdir basics_ros2_multithreading/scripts
touch basics_ros2_multithreading/scripts/function.py
touch basics_ros2_multithreading/scripts/callback_function.py
touch basics_ros2_multithreading/scripts/callback_spinonce_function.py
touch basics_ros2_multithreading/scripts/face_searcher.py

mkdir basics_ros2_multithreading/launch
touch basics_ros2_multithreading/launch/callback_function.launch.py
touch basics_ros2_multithreading/launch/face_searcher.launch.py

# Download extra files to make your life easier
git clone https://bitbucket.org/theconstructcore/basics_ros2_extra_material.git
cp basics_ros2_extra_material/unit_callbacks_and_threads/haarcascade_frontalface.xml basics_ros2_multithreading/scripts/haarcascade_frontalface.xml
cp basics_ros2_extra_material/unit_callbacks_and_threads/deepbot_movehead.py basics_ros2_multithreading/scripts/deepbot_movehead.py
rm -rf basics_ros2_extra_material
```

#### 2. 修改CMakelists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(basics_ros2_multithreading)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# We add this to be able to import scripts form other scripts of our package
# Also to execute launch files in the launch folder
install(DIRECTORY
  scripts
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Make the scripts executable with ros2 run basics_ros2_multithreading NAME
install(PROGRAMS
  scripts/function.py
  scripts/callback_function.py
  scripts/callback_spinonce_function.py
  scripts/face_searcher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

### 5.3 为什么要使用CallBacks？

- What is a callback (`callback`)?
- For that, learn what a function is.
- Start with the simplest ROS2 program.
- Add the following code to the file `function.py`.

  **function.py**

```python
#!/usr/bin/env python3
import rclpy
import time

def robot_message(text, robot_name="Robot-1"):
    print(robot_name+": "+text)


def main(args=None):
    rclpy.init(args=args)
    period = 1.0
    robot_message(text="Robot Booting Up...")
    time.sleep(period)
    robot_message(text="Robot Ready...")
    time.sleep(period)
    robot_message(text="Robot ShuttingDown...")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

- Build and execute

  ```
  cd ~/ros2_ws/
  source /home/simulations/ros2_sims_ws/install/setup.bash
  colcon build --packages-select basics_ros2_multithreading
  source install/setup.bash
  ros2 run basics_ros2_multithreading function.py
  ```

- This was straightforward:
  - You have a `main` function that performs the `main` task of your script.
  - In this case, the `main` task is to give the **robot's internal status messages**.
  - You have a `function` named `robot_message` that prints a message in the console when called.
- This script has only **ONE** task to perform:
  - Giving the **robot's internal status messages**.

- If I need now for this script to perform:
  - **ONE** **MAIN TASK**: Continue giving the robots internal status messages.
  - 继续提供机器人内部状态消息。
  - **ANOTHER** **secondary TASK**: Count the seconds the robot has been operational.
  - 计算机器人已经运行的秒数。
- How would you do it?
- You would need:
  - *A class to store and access the time passed.*
  - 一个可以存储和访问时间的类。
  - *The primary task function is to print the robot status messages.*
  - 主要任务function是打印机器人状态消息。
  - *A function that increments the time passed.*
  - 递减时间的函数。
- Modify the `function.py` code like so:

```python
#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node

class RobotStatus(Node):
    def __init__(self):
        super().__init__('robot_status')
        self.time_robot_on = 0.0
        
    def robot_message(self, text, robot_name="Robot-1"):
        self.get_logger().info(robot_name+": "+text)

    def timer_counter(self, time_passed):
        self.time_robot_on += time_passed
        self.get_logger().info("Updated Time Robot On="+str(self.time_robot_on))

    def main_task(self):
        period = 1.0
        self.robot_message(text="Robot Booting Up...")
        time.sleep(period)
        self.timer_counter(time_passed=period)

        self.robot_message(text="Robot Ready...")
        time.sleep(period)
        self.timer_counter(time_passed=period)

        self.robot_message(text="Robot ShuttingDown...")

def main(args=None):
    rclpy.init(args=args)
    robot_status_node = RobotStatus()
    robot_status_node.main_task()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

- Build and execute

```
cd ~/ros2_ws/
source /home/simulations/ros2_sims_ws/install/setup.bash
colcon build --packages-select basics_ros2_multithreading
source install/setup.bash
ros2 run basics_ros2_multithreading function.py
```

- 这个脚本激活了两个函数:
  - Execute the function `robot_message`
  - Execute the function `timer_counter`.

- 但是，这种执行函数的方式有一些局限性和缺点：
  - 脚本一直在运行.
  - 在许多事情发生的系统中，这可能很难管理

有一种不同的执行函数的方式，那就是**通过事件**。

通过事件执行函数意味着你只有在触发了一个事件时，也就是在某事发生时，才会执行一个函数。

在ROS2中，事件可以是：

- 传入的传感器消息。
- 服务请求。
- 定时器事件。
- 行动目标。

例如：

你的 `main_task` 函数可以由**timer event** 触发，因为它每隔1.0秒执行一些新的任务。

 同样，`timer_counter` 也可以由定时器事件触发。

 在机器人系统中，事件对于在特定条件发生或在特定时间自动执行函数至关重要。

由事件触发的函数被称为：

**回调函数（Callback Functions）**。

 **callback_function.py**

```python
#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node

class RobotStatus(Node):
    def __init__(self):
        super().__init__('robot_status')
        self.time_robot_on = 0.0
        self.timer_period = 0.1
        self.sleep_timer_counter = 0.0
        self.robot_status = ["Robot Booting Up...",
                             "Robot Ready...",
                             "Robot ShuttingDown..."]
        self.main_task_period = 1.0
        self.sleep_time_main_task = 0.0
    
        # def create_timer(
        #     self,
        #     timer_period_sec: float,
        #     callback: Callable,
        #     callback_group: CallbackGroup = None,
        #     clock: Clock = None,
        # ) -> Timer:

    
        self.create_timer(self.timer_period, self.timer_counter)
        self.create_timer(self.main_task_period, self.main_task)
        
    def robot_message(self, text, robot_name="Robot-1"):
        self.get_logger().info(robot_name+": "+text)

        
    # 回调函数1  
    def timer_counter(self):
        self.time_robot_on += self.timer_period
        self.get_logger().info("Updated Time Robot On="+str(self.time_robot_on))
        time.sleep(self.sleep_timer_counter)
        self.get_logger().info("Updated Time Robot On="+str(self.time_robot_on))

        
    # 回调函数2
    def main_task(self):

        if len(self.robot_status)>=1:
            status = self.robot_status.pop(0)
            self.robot_message(text=status)
        else:
            status = "ShuttingDown"

        

        if "ShuttingDown" in status:
            self.get_logger().info('Shutting down node...')
            self.destroy_node()
            rclpy.shutdown()
        else:
            self.get_logger().info('Continue....')
            time.sleep(self.sleep_time_main_task)

def main(args=None):
    rclpy.init(args=args)
    robot_status_node = RobotStatus()
    rclpy.spin(robot_status_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```



- 你定义了两个函数，`main_task` 和 `timer_counter`，并将它们设置为**回调函数**（CALLBACKS）。
- 这些 **Callbacks** 将由两个不同的定时器触发：
  - `self.timer_period`：定时器每隔 0.1 秒触发一次 `timer_counter` 函数。
  - `self.main_task_period`：定时器每隔 1.0 秒触发一次 `main_task` 函数。
- 你需要区分两个**回调周期**（CALLBACK PERIODS），`self.timer_period` 和 `self.main_task_period`，这是你调用回调函数的周期。
- **函数处理时间**（FUNCTION PROCESSING TIME）是每个回调函数执行并完成所需的时间。
- 每个函数，`timer_counter` 和 `main_task`，内部都有模拟处理时间的休眠操作，赋予它们不同的**函数处理时间**。
- 添加这种模拟处理时间是因为，在机器人感知或类似的功能中，函数通常需要一定时间来完成。
- 目前，将它们设置为瞬时完成：
  - **`self.sleep_timer_counter = 0.0`**
  - **`self.sleep_time_main_task = 0.0`**
- 每次定时器事件触发回调时，该回调会被放入一个**回调队列**（CALLBACK QUEUE）中。
- 然后，`rclpy.spin` 会访问这个**回调队列**，并按照**先进先出**（FIFO）的顺序处理每个回调。.

**callback_function.launch.py**

```python
import launch
import launch_ros.actions
from tracetools_launch.action import Trace


def generate_launch_description():
    
    # Trace
    trace = Trace(
        session_name='callback_function'
    )
    # Nodes
    callback_function_node = launch_ros.actions.Node(
        package='basics_ros2_multithreading',
        executable='callback_function.py',
        arguments=[],
        output='screen',
    )

    
    ##############################################################
    return launch.LaunchDescription([
        trace,
        callback_function_node
    ])
	##############################################################
```

- **NOTE** that you are adding a strange *trace* ClassNode launch.
- This will allow you to see the callback sequence in time of the node executed in the script `callback_function.py`.
- All the info will be stored in a **tracing folder** named: `~/.ros/tracing/callback_function`.

通过引入 `ClassNode launch` 跟踪功能，你可以详细追踪并分析节点在执行过程中回调函数的调用顺序和时间



- Build and execute

```
cd ~/ros2_ws/
source /home/simulations/ros2_sims_ws/install/setup.bash
colcon build --packages-select basics_ros2_multithreading
source install/setup.bash

rm -rf ~/.ros/tracing/callback_function/
ros2 launch basics_ros2_multithreading callback_function.launch.py
```

通过使用回调机制，现在可以每0.1秒调用一次`timer_counter`函数。

这比之前的非回调版本有了改进，因为之前的版本只能以单一频率执行。

- We provide you with a script to do exactly that, draw the callback executions through time:

- Now, visualize how these callbacks are executed through time.
- We provide you with the script to draw the callback executions through time:

```
cd ~/ros2_ws/
source /home/simulations/ros2_sims_ws/install/setup.bash
ros2 run ros2_callback_visualiser analise_trace_calback_function.py
```

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_ROS2/images/callback_vis_1.png)

正如您所看到的，每个回调都以适当的间隔进行调用， `timer_counter` 的间隔为0.1，`main_task`回调函数的间隔为1.0。

时间安排是完美的，回调是完美的。但是，那是因为他们几乎没有任何执行每个回调的事情。他们是瞬时的。

然而，事实并非如此。像perception functions一样，函数需要时间来处理人脸或检测物体。



- Modify our script so that the functions`timer_counter`and 1.0 the`main_task`
- take some time to process:
  - **`self.sleep_timer_counter = 1.0`**
  - **`self.sleep_time_main_task = 1.5`**

- Build and execute
- Note that we removed the tracing folder in case we had executed the command already

```
cd ~/ros2_ws/
source /home/simulations/ros2_sims_ws/install/setup.bash
colcon build --packages-select basics_ros2_multithreading
source install/setup.bash


rm -rf ~/.ros/tracing/callback_function/
ros2 launch basics_ros2_multithreading callback_function.launch.py
```

- Review the **callback execution timeline**.
- **REMEMBER**, before launching the `analise_trace_calback_function.py`, please close the previous **window with the graph**; otherwise, it might cause issues.

```
cd ~/ros2_ws/
source /home/simulations/ros2_sims_ws/install/setup.bash
ros2 run ros2_callback_visualiser analise_trace_calback_function.py
```

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_ROS2/images/callback_vis_2.png)

- **Not all the callbacks that should have been executed have been executed.**
- It seems that it has executed one or the other.
- `timer_counter` has a period of *0.1 seconds*, so you should have executed more than **4** times in *8 seconds*.
- Why is this happening?
- The issue here is that it is **OBVIOUS NOW**, with longer execution times, that ROS2 nodes, by default, **CAN ONLY EXECUTE ONE CALLBACK at a time**.
- So, you cannot execute the callbacks at the right times if any previous callback takes more than **0.0** seconds to be executed.

并非所有应该被执行的回调函数都已经执行。看起来它只执行了一部分。

`timer_counter`的周期是0.1秒，因此在8秒内它应该执行了超过4次。但是，为什么会发生这种情况呢？

这里的问题很明显，当执行时间较长时，ROS2节点默认情况下一次只能执行一个回调。

因此，如果任何先前的回调执行时间超过0.0秒，就无法在正确的时间执行回调。

如果存在执行时间较长的回调函数，它可能会阻塞其他回调函数的执行，导致一些回调函数不能按预期频率执行。

**WHY ONLY ONE callback at a time?**

- This happens with **ANY CALLBACKS IN ROS2**.
- It happens with *timer callbacks, topic subscription callbacks, service call callbacks, and action server callbacks*.

- See a more robotics-like example.
- You have the following simulation that you can see in the **SIMULATION WINDOW**:

- Add the code for the script `face_search.py` you created in the **setup phase** of the unit.
- This program will have the following callbacks:
  - An RGB camera image subscriber callback.
  - A service callback that, when called, will search for human faces.

 **face_searcher.py**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory
import os
from geometry_msgs.msg import Twist  # Import for velocity commands
from rclpy.duration import Duration
import time

class FaceDetectorNode(Node):
    def __init__(self, image_topic_name = "/deepmind_robot1/deepmind_robot1_camera/image_raw"):
        super().__init__('face_detector_node')
        self._image_topic_name = image_topic_name
        
        ################## 订阅相机流 #######################
        # module(Interface，接收的数据类型)：Image
        # Topic：_image_topic_name
        # callbacks：listener_callback
        self.subscription = self.create_subscription(
            Image,
            self._image_topic_name,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
       
   		################### 创建service #####################
        # Service Server type：trigger  
        # name：detect_faces
        # callbacks：detect_faces_callback
        self.srv = self.create_service(Trigger, 'detect_faces', self.detect_faces_callback)
        self.bridge = CvBridge()
        self.last_image = None

        
       
        ######################## Publisher ##################
        # message type：Twist
        # publish的目标：/cmd_vel
        # queue size 10
        # Publisher for cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        package_name = 'basics_ros2_multithreading'  # Replace with your package name
        package_path = get_package_share_directory(package_name)
        haar_cascade_file_path = os.path.join(package_path,"scripts/haarcascade_frontalface.xml")
        print(f"Package Path: {haar_cascade_file_path}")


        self.face_cascade = cv2.CascadeClassifier(haar_cascade_file_path)

        
        ########################## listener_callback ###############################
        # 每次接收到相机流，使用cvbridge转化成 cv frame
    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        self.last_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

        
        #################### detect_faces_callback #############################
        # 对图像进行人脸检测
    def detect_faces_callback(self, request, response):
        start_time = self.get_clock().now()
        duration_10_sec = Duration(seconds=10)
        

        face_found = False

        # We start moving the robot
        self.publish_velocity(linear=0.0, angular=-0.5)
        delta_time = self.get_clock().now() - start_time
        self.get_logger().info('Delta Time='+str(delta_time))

        while delta_time < duration_10_sec and not face_found:
            delta_time = self.get_clock().now() - start_time
            self.get_logger().info('Delta Time='+str(delta_time))

            if self.last_image is not None:
                self.get_logger().info("Image....")
                gray = cv2.cvtColor(self.last_image, cv2.COLOR_BGR2GRAY)
                self.get_logger().info("Faces Processing....")
                faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)
                
                if len(faces) > 0:
                    self.publish_velocity(linear=0.0, angular=0.0)
                    self.get_logger().info("Face Found....")
                    face_found = True
                    self.get_logger().info('face_found='+str(face_found))
                    for (x, y, w, h) in faces:
                        cv2.rectangle(self.last_image, (x, y), (x+w, y+h), (255, 0, 0), 2)
                    
                    cv2.imshow("Faces", self.last_image)
                    cv2.waitKey(0)

                    break  # Exit the loop as a face is found
            self.get_logger().info("Sleep....")
            time.sleep(0.1)
            self.get_logger().info("End Sleep....")

        # We stop the robot
        self.publish_velocity(linear=0.0, angular=0.0)
        self.get_logger().info("End...")
        response.success = face_found
        response.message = 'Processed latest image'
        return response
    
    def publish_velocity(self, linear, angular):
        # Create Twist message
        vel_msg = Twist()
        vel_msg.linear.x = linear
        vel_msg.angular.z = angular
        # Publish the message
        self.cmd_vel_publisher.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    face_detector_node = FaceDetectorNode()
    rclpy.spin(face_detector_node)
    face_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

- This script uses `haarcascade` with OpenCV to detect faces in images.

`Haar Cascade` 是一种由 Paul Viola 和 Michael Jones 在 2001 年提出的用于对象检测的计算机视觉算法。这种方法特别擅长进行面部检测，尽管它也可以用于检测其他类型的物体。`Haar Cascade` 使用机器学习方法，训练一个“级联函数”来进行快速和有效的面部检测。

- The part that concerns you now is the **CALLBACK** this node has:

  - `listener_callback`: Called when a new message in the camera topic `/deepmind_robot1/deepmind_robot1_camera/image_raw` is published. In this case, the **EVENT** is not time but the publication of a message in that topic.
  - `detect_faces_callback`: This callback will be called when the service `detect_faces` is called.
  
- Therefore, you have **TWO** callbacks that can be called.

- **NOTE **that the` detect_faces_callback` will not finish
  **UNTIL**:
  - **It has found a face**.
  - Or **10 seconds have passed** without finding a face.

**face_searcher.launch.py**



```python
import launch
import launch_ros.actions
from tracetools_launch.action import Trace


def generate_launch_description():
    
    # Trace
    trace = Trace(
        session_name='face_searcher'
    )
    # Nodes
    face_searcher_node = launch_ros.actions.Node(
        package='basics_ros2_multithreading',
        executable='face_searcher.py',
        arguments=[],
        output='screen',
    )

    return launch.LaunchDescription([
        trace,
        face_searcher_node
    ])
```

- This launch file follows the same structure as the `callback_function.launch.py`.

- Now, execute the `face_searcher.launch.py`.
- This will register, like you already did, the timeline of callbacks.
- Build and execute

```
cd ~/ros2_ws/
source /home/simulations/ros2_sims_ws/install/setup.bash
colcon build --packages-select basics_ros2_multithreading
source install/setup.bash

rm -rf ~/.ros/tracing/face_searcher/
GDK_SYNCHRONIZE=1 ros2 launch basics_ros2_multithreading face_searcher.launch.py
```

- The callback for the image will be called because the robot camera publishes the images continuously.
- Before calling the service `detect_face`, review what is seen right now by the robot:
- 在调用Service detect_face之前，请查看机器人现在看到的内容：

```
cd ~/ros2_ws/
source install/setup.bash
ros2 run rqt_image_view rqt_image_view /deepmind_robot1/deepmind_robot1_camera/image_raw
```

- As you can see, no person appears in the image at the robot's starting position.
- This is done on purpose because this way, the robot **should start turning until it detects a face.**
- Now, call the service `/detect_faces` to start the **FACE DETECTION SEARCH**.
- 激活service，service会Publish信息

```
cd ~/ros2_ws/
source install/setup.bash
ros2 service call /detect_faces std_srvs/srv/Trigger "{}
"
```

- So, the `detect_faces_callback` has been called correctly.
- **BUT the robot does not stop until the 10 seconds have passed**.
- Is the **FACE DETECTION NOT WORKING**?
- Move the robot manually to face the girl before calling the service again.
- **TURN THE ROBOT** with the `l` or `j` keys on the keyboard.
- 键盘控制，使相机指向女孩的脸

```
cd ~/ros2_ws/
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

- Check that the images published have the girl appearing:

- 查看相机流，是否看到了脸

  ```
  cd ~/ros2_ws/
  source install/setup.bash
  ros2 run rqt_image_view rqt_image_view /deepmind_robot1/deepmind_robot1_camera/image_raw
  ```

- Right! Now, call the service again and see if it detects the **person** or if it happens the same as before:
- call server 检测人脸

```
cd ~/ros2_ws/
source install/setup.bash
ros2 service call /detect_faces std_srvs/srv/Trigger "{}
"
```

- Now it has been detected!
- In the Graphical Tools, a window should appear showing the detection.
- When you close that window, the service call should receive a *response* **`True`**, reiterating that a face was found.
- And the robot stopped when the detection was made.

- You can get some insight by having a look at the `callback timelines` executing:



```
cd ~/ros2_ws/
source /home/simulations/ros2_sims_ws/install/setup.bash
ros2 run ros2_callback_visualiser analise_trace_face_searcher_callbacks.py
```





![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_ROS2/images/tracefacedetectnocallbacks.png)

- What you are seeing are both callbacks `listener_callback` and `detect_faces_callback`.
- `listener_callback` is being called regularly because that is the rate at which the camera publishes new images.
- However, then, something happens. **THEY STOP**.
- Remember that ROS2 nodes, by default, **ONLY CAN EXECUTE ONE CALLBACK at a time**?
- Now that the `detect_faces_callback` has been called, **NO OTHER CALLBACK** can be called until it finishes.
- But it will not finish until it detects a person or 10 seconds pass.
- **BUT WHY DOES IT NOT DETECT THE PERSON** when you start from a position where the robot was not seeing the person before calling the service?

- Here, you see that in the section prior to the service call, you have the same image, more or less, because the robot is still.
- When you enter the `detec_faces_callback`, the robot **MOVES** and the images change in the camera.
- **However**, the image variable used inside the `detect_faces_callback` (`self.last_image`) you use is updated by the `listener_callback`... **AND THAT CALLBACK IS NOT CALLED anymore**.
- This means that through all the processing of the `detect_faces_callback`, the image (`self.last_image`) is **EXACTLY THE SAME**.
- **THAT IS A REAL PROBLEM**, a robot that when moves, the **SENSOR DATA is not updated inside the control script**... THAT IS **BAD**.
- The image variable (`self.last_image`) will only be **UPDATED** when you start to call the callback `listener_callback`, as shown in the diagram animation.

- The solution would be to **EXECUTE BOTH CALLBACKS SIMULTANEOUSLY**, updating the image variable (`self.last_image`) while the `detect_faces_calback` is executing.
- To do that, you have to introduce a **new concept**:  MULTI-THREADING IN ROS2



- `listener_callback`定期被调用，因为这是摄像头发布新图像的频率。
- 当`detect_faces_callback`被调用时，根据ROS 2的默认行为，没有其他回调可以被执行，直到它完成。如果在检测到人或过了10秒之前`detect_faces_callback`没有完成，它就不会结束。
- 如果机器人在调用服务前没有看到人，那么为什么它不能检测到人呢？这是因为，在`detect_faces_callback`执行期间，机器人移动了，摄像头的图像发生了变化。
- 但是，用于检测人脸的图像变量（`self.last_image`）是由`listener_callback`更新的，而这个回调在`detect_faces_callback`执行期间不会被调用，导致`self.last_image`在整个`detect_faces_callback`处理期间保持不变。



这就是一个实际问题：一个移动的机器人，在其控制脚本内部的传感器数据不会更新，这是不好的。

解决方案是同时执行两个回调，当`detect_faces_callback`执行时，同时更新图像变量（`self.last_image`）。

为了做到这一点，需要引入一个新概念：ROS 2中的多线程（MULTI-THREADING）。

### 5.4 SPIN **ONCE**

- One issue when executing the `callback_function.py` script was that you had to press `CTRL+C` to finish it.
- This happens because rclpy.spin() is still running and waiting for callbacks to finish even after you call rclpy.shutdown()
- Here, you have the code to do the same thing you did but with a little more complex code using `spin_once`.
- **SPIN_ONCE** processes all the callbacks in the queue, but instead of doing it continuously like `spin()` does, it does it only once. 
- Spin_Once处理队列中的所有回调，但不像Spin（）那样连续进行一次，而是仅执行一次。
- It is that simple. That way, the script will not get stuck there forever.
- In these examples, you added some **TIME** for each callback to be processed.

**callback_spinonce_function.py**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class RobotStatus(Node):
    def __init__(self):
        super().__init__('robot_status')
        self.time_robot_on = 0.0
        self.timer_period = 0.1  # Timer period in seconds
        self.robot_status = ["Robot Booting Up...",
                             "Robot Ready...",
                             "Robot ShuttingDown..."]
        self.main_task_period = 1.0  # Main task period in seconds
        self.shutdown_flag = False  # Flag to indicate shutdown
        
        self.create_timer(self.timer_period, self.timer_counter)
        self.create_timer(self.main_task_period, self.main_task)
        
    def robot_message(self, text, robot_name="Robot-1"):
        self.get_logger().info(f"{robot_name}: {text}")

    def timer_counter(self):
        self.time_robot_on += self.timer_period
        self.get_logger().info(f"Updated Time Robot On={self.time_robot_on:.1f}")

    def main_task(self):
        if self.shutdown_flag:
            return  # Skip the main task if shutting down

        import time
        time.sleep(10)

        status = self.robot_status.pop(0)
        self.robot_message(text=status)

        if "ShuttingDown" in status:
            self.get_logger().info('Shutting down node...')
            self.shutdown_flag = True  # Set the shutdown flag
        else:
            self.get_logger().info('Continue....')

def main(args=None):
    rclpy.init(args=args)
    robot_status_node = RobotStatus()

    # Use spin_once inside a loop instead of spin
    while rclpy.ok():
        rclpy.spin_once(robot_status_node)  # Handle callbacks
        if robot_status_node.shutdown_flag:
            robot_status_node.destroy_node()  # Destroy the node
            rclpy.shutdown()  # Shutdown ROS2 client library
            break  # Break out of the loop to end the script

    robot_status_node.get_logger().info('Node has been shut down.')

if __name__ == '__main__':
    main()
```



- You can see that you replace the following:

```python
robot_status_node = RobotStatus()
rclpy.spin(robot_status_node)
rclpy.shutdown()
```

- By:

```python
robot_status_node = RobotStatus()
while rclpy.ok():
    rclpy.spin_once(robot_status_node)  # Handle callbacks
    if robot_status_node.shutdown_flag:
        robot_status_node.destroy_node()  # Destroy the node
        rclpy.shutdown()  # Shutdown ROS2 client library
        break  # Break out of the loop to end the script
```

### 5.5 Summary

- You have learned that in ROS2, you can call **functions** using **EVENTS**, known as **CALLBACKS**.
- *Timer events, topic subscriber events, service events, and action events* can trigger callbacks.
- By default, **ROS2 nodes** can only execute **ONE CALLBACK at a time**.
- This brings issues in real robotic systems needed to execute **MULTIPLE CALLBACKS** at the same time.
- The solution is **MULTITHREADING in ROS2**.

## 6  Multithreading

### 6.1  Objective

如果要创建ROS2程序，以从多个传感器中检索信息或同时执行多个任务，则必须了解多线程。

### 6.2  Setup

- First, check that you have the previous unit package `basics_ros2_multithreading` to house all the code you generate in this unit.
- You will also **create all the files** you will be writing the code for so you can concentrate on the actual concepts inside them.

```cmd
cd ~/ros2_ws/src
ls basics_ros2_multithreading

touch basics_ros2_multithreading/scripts/face_searcher_multithreading.py
touch basics_ros2_multithreading/scripts/face_searcher_multithreading_callbackgroups.py
touch basics_ros2_multithreading/scripts/yello_tshirt_detector.py

touch basics_ros2_multithreading/launch/face_searcher_multithreading.launch.py
touch basics_ros2_multithreading/launch/face_searcher_multithreading_callbackgroups.launch.py
touch basics_ros2_multithreading/launch/yello_tshirt_detector.launch.py


git clone https://bitbucket.org/theconstructcore/basics_ros2_extra_material.git
cp basics_ros2_extra_material/unit_callbacks_and_threads/haarcascade_frontalface.xml basics_ros2_multithreading/scripts/
rm -rf basics_ros2_extra_material
```

Cmakelists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(basics_ros2_multithreading)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# We add this to be able to import scripts form other scripts of our package
# Also to execute launch files in the launch folder
install(DIRECTORY
  scripts
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Make the scripts executable with ros2 run basics_ros2_multithreading NAME
install(PROGRAMS
  scripts/function.py
  scripts/callback_function.py
  scripts/callback_spinonce_function.py
  scripts/face_searcher.py
  scripts/face_searcher_multithreading.py
  scripts/face_searcher_multithreading_callbackgroups.py
  scripts/yello_tshirt_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```



### 6.3  Why You Need Multithreading

- In the previous unit, you saw that you need **MULTITHREADING in ROS2** to execute **MORE THAN ONE CALLBACK SIMULTANEOUSLY**. 
- 同时运行不止一个Callbacks
- How can you add **multithreading to the example you did** so that you can execute both callbacks (`listener_callback` and `detect_faces_callback`) simultaneously?
- We will introduce a new concept: **MULTITHREADED EXECUTORS** （多线程执行者）

- To add it to your previous code, the only thing you will have to change is the `__main__` function, like so:

#### OLD CODE SECTION

```python
# REST OF THE CODE

def main(args=None):
    rclpy.init(args=args)
    face_detector_node = FaceDetectorNode()
    rclpy.spin(face_detector_node)
    face_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### NEW CODE SECTION

```python
from rclpy.executors import MultiThreadedExecutor

# REST OF THE CODE IS IDENTICAL


def main(args=None):
    rclpy.init(args=args)
    face_detector_node = FaceDetectorNode()
    
    # Use MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(face_detector_node)
    
    try:
        executor.spin()
    finally:
        face_detector_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

- The only difference is that now the node `face_detector_node` is managed, not directly by the `rclpy.spin`, but by the `executor` object.
- 不同点在于，原来的`face_detector_node`这个节点是由`rclpy.spin`激活的，现在我们添加了一个`MultiThreadedExecutor`的对象`executor`，再用这个`executor`执行节点，然后spin`executor`
- This `MultiThreadedExecutor` allows you to select **HOW MANY THREADS** you need.
- 对象参数：线程数`num_threads`
- Typically, you need **ONE THREAD PER CALLBACK** to guarantee that both can be executed simultaneously.
- 一般的，每一个线程分配一个回调函数
- The rest is essentially the same structure:
  - As always, use `spin` to process all the callbacks in the queue.
  - When you are finished, `destroy` the nodes managed by the executor.



- If you want to know how many threads you can have in your system. Try this in a Python interpreter:

- In the current course system, it will most probably give **8**.

```
python3
>>> import multiprocessing
>>> multiprocessing.cpu_count()
8
```

- Add the code with the changes to a different script to compare:

  **face_searcher_multithreading.py**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory
import os
from geometry_msgs.msg import Twist  # Import for velocity commands
from rclpy.duration import Duration
import time
from rclpy.executors import MultiThreadedExecutor

class FaceDetectorNode(Node):
    def __init__(self, image_topic_name = "/deepmind_robot1/deepmind_robot1_camera/image_raw"):
        super().__init__('face_detector_node')
        self._image_topic_name = image_topic_name
        self.subscription = self.create_subscription(
            Image,
            self._image_topic_name,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.srv = self.create_service(Trigger, 'detect_faces', self.detect_faces_callback)
        self.bridge = CvBridge()
        self.last_image = None

        # Publisher for cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        package_name = 'basics_ros2_multithreading'  # Replace with your package name
        package_path = get_package_share_directory(package_name)
        haar_cascade_file_path = os.path.join(package_path,"scripts/haarcascade_frontalface.xml")
        print(f"Package Path: {haar_cascade_file_path}")


        self.face_cascade = cv2.CascadeClassifier(haar_cascade_file_path)

    def listener_callback(self, data):
        self.get_logger().info('MULTITHREADING Receiving video frame')
        self.last_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

    def detect_faces_callback(self, request, response):
        start_time = self.get_clock().now()
        duration_10_sec = Duration(seconds=10)
        

        face_found = False

        # We start moving the robot
        self.publish_velocity(linear=0.0, angular=-0.3)
        delta_time = self.get_clock().now() - start_time
        self.get_logger().info('Delta Time='+str(delta_time))

        while delta_time < duration_10_sec and not face_found:
            delta_time = self.get_clock().now() - start_time
            self.get_logger().info('Delta Time='+str(delta_time))

            if self.last_image is not None:
                self.get_logger().info("Image....")
                gray = cv2.cvtColor(self.last_image, cv2.COLOR_BGR2GRAY)
                self.get_logger().info("Faces Processing....")
                faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)
                
                if len(faces) > 0:
                    self.publish_velocity(linear=0.0, angular=0.0)
                    self.get_logger().info("Face Found....")
                    face_found = True
                    self.get_logger().info('face_found='+str(face_found))
                    for (x, y, w, h) in faces:
                        cv2.rectangle(self.last_image, (x, y), (x+w, y+h), (255, 0, 0), 2)
                    
                    cv2.imshow("Faces", self.last_image)
                    cv2.waitKey(0)

                    break  # Exit the loop as a face is found
            self.get_logger().info("Sleep....")
            time.sleep(0.1)
            self.get_logger().info("End Sleep....")

        # We stop the robot
        self.publish_velocity(linear=0.0, angular=0.0)
        self.get_logger().info("End...")
        response.success = face_found
        response.message = 'Processed latest image'
        return response
    
    def publish_velocity(self, linear, angular):
        # Create Twist message
        vel_msg = Twist()
        vel_msg.linear.x = linear
        vel_msg.angular.z = angular
        # Publish the message
        self.cmd_vel_publisher.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    face_detector_node = FaceDetectorNode()
    
    # Use MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(face_detector_node)
    
    try:
        executor.spin()
    finally:
        face_detector_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

- You also need a launch file that allows you to **trace** this new script with multithreading:
- 通过多线程跟踪此新脚本

  **face_searcher_multithreading.launch.py**

```python
import launch
import launch_ros.actions
from tracetools_launch.action import Trace


def generate_launch_description():
    
    ##############################################################
    # Trace
    trace = Trace(
        session_name='face_searcher'
    )
    ##############################################################
    
    # Nodes
    face_searcher_multithreading_node = launch_ros.actions.Node(
        package='basics_ros2_multithreading',
        executable='face_searcher_multithreading.py',
        arguments=[],
        output='screen',
    )

    return launch.LaunchDescription([
        trace,
        face_searcher_multithreading_node
    ])
```

- Now, execute the `face_searcher_multithreading.launch.py`.
- This will register the timeline of callbacks in the same way you already did.

- Build and execute

```
cd ~/ros2_ws/
source /home/simulations/ros2_sims_ws/install/setup.bash
colcon build --packages-select basics_ros2_multithreading
source install/setup.bash

rm -rf ~/.ros/tracing/face_searcher/
GDK_SYNCHRONIZE=1 ros2 launch basics_ros2_multithreading  face_searcher_multithreading.launch.py
```

- Now, call the service `/detect_faces` to start the **FACE DETECTION SEARCH**.

Terminal 2:

```
cd ~/ros2_ws/
source install/setup.bash
ros2 service call /detect_faces std_srvs/srv/Trigger "{}
"
```

- **The same issue is happening**!
- Check the callback timeline:

- **REMEMBER** that this script has to be terminated manually by pressing `CTRL+C`.
- **IMPORTANT**: before launching the `analise_trace_face_searcher_callbacks.py`, please close the previous **window with the graph**; otherwise, it might cause issues.

```
cd ~/ros2_ws/
source /home/simulations/ros2_sims_ws/install/setup.bash
ros2 run ros2_callback_visualiser analise_trace_face_searcher_callbacks.py
```



- To understand this, you need to introduce a **new concept**:**Callback groups (`Callback groups`)**



### 6.4  Why You Need Callback Groups

- You have seen that **BY DEFAULT**, each node you start in ROS2 has **ONE thread**.
- This is the reason you can only execute **ONE CALLBACK AT A TIME**.

- ROS2 nodes also have what is called **Callback Groups**.
- There are **TWO TYPES** of callback groups:
  - `ReentrantCallbackGroup`
  - `MutuallyExclusiveCallbackGroup`
- **By default**, ROS2 nodes have only **ONE** callback group, and its type is `MutuallyExclusiveCallbackGroup`.
- Therefore, in a default ROS2 node. All the callbacks defined inside that node will be placed inside this `MutuallyExclusiveCallbackGroup`.
- Here, you can see the morphology of your `face_detector_node` in the original file `face_searcher.py` (that you created in the previous unit) with the following:
  - One Thread
  - One callback group
  - Two callbacks

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_ROS2/images/MultithreadingSImple.png)

- Here, you can see the morphology of your `face_detector_node` that you modified to have two threads in the file `face_searcher_multithreading.py`:
  - Two threads, you added them using `Executors`.
  - One callback group
  - Two callbacks

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_ROS2/images/MultithreadingSImple2.png)

- Why did this morphology in file `face_searcher_multithreading.py`, with **TWO threads**, NOT WORK?
- Why are the callbacks still executed **ONE AT A TIME**?
- To understand this, you need to know the difference between a `ReentrantCallbackGroup` and `MutuallyExclusiveCallbackGroup`.



- The **MAIN difference** between these two types of callback groups is the **CONCURRENCY**.
- **CONCURRENCY** regulates whether you can execute several callbacks simultaneously.
- 这两种类型的回调组之间的主要区别是并发。并发调节您是否可以同时执行多个回调。



#### `ReentrantCallbackGroup`:

- Concurrency:

  - Allows **multiple callbacks in the same group to be executed simultaneously**.

  - This means that if one callback is already running, another callback can start executing without waiting for the first one to finish.

    

- Use Case:

  - Suitable for scenarios where callbacks **do not interfere with each other and can safely run simultaneously**.

  - This is often used in applications where high throughput or responsiveness is needed, and the callbacks are designed to be independent.

    

- Example:

  - If you have multiple sensors and each sensor has its own callback to process data, these callbacks can run concurrently without affecting each other.

#### `MutuallyExclusiveCallbackGroup`:

- Concurrency:

  - Ensures that **only one callback in the group can be executed at a time**.

  - When one callback is running, all other callbacks in the group are blocked until the running callback finishes.

    

- Use Case:

  - Ideal for scenarios where callbacks **share resources or need to be executed in a specific sequence**.

  - This prevents race conditions and ensures data consistency when callbacks interact with shared resources.

    

- Example:

  - If a single resource (like a hardware component or a shared variable) is accessed by multiple callbacks, using a MutuallyExclusiveCallbackGroup ensures that only one callback interacts with the resource at any given time.



两种ROS 2中的回调组（Callback Group）类型，它们之间的主要区别在于并发性（Concurrency）。

1. **ReentrantCallbackGroup（可重入回调组）:**
   - **并发性：** 允许同一组中的多个回调同时执行。这意味着如果一个回调已经在运行，另一个回调可以开始执行，而无需等待第一个回调完成。
   - **用例：** 适用于**回调之间不相互干扰且可以安全同时运行**的场景。通常在需要高吞吐量或响应性的应用中使用，其中回调被设计为相互独立。
   - **示例：** 如果有多个传感器，每个传感器都有自己的回调来处理数据，这些回调可以在不影响彼此的情况下同时运行。
2. **MutuallyExclusiveCallbackGroup（互斥回调组）:**
   - **并发性：** 确保组中只能同时执行一个回调。当一个回调正在运行时，组中的所有其他回调都会被阻塞，直到运行的回调完成。
   - **用例：** 适用于**回调共享资源或需要按特定顺序执行**的场景。这可以防止竞态条件，并确保当回调与共享资源交互时数据的一致性。
   - **示例：** 如果多个回调访问单一资源（例如硬件组件或共享变量），使用互斥回调组可以确保在任何给定时间只有一个回调与资源交互。

总的来说，`ReentrantCallbackGroup`适用于相互独立的回调，可以同时执行，而`MutuallyExclusiveCallbackGroup`适用于需要互斥执行的回调，以确保对**共享资源**的访问是有序的。这两者提供了在ROS 2中处理不同并发需求的方式。

#### What are race conditions?

- In Python, when a variable is accessed simultaneously by two different callbacks (or threads), it can lead to concurrency issues, commonly known as **race conditions**.

- **Race conditions** occur when multiple threads or processes access shared data simultaneously, and at least one of them modifies the data.

- This can lead to unpredictable and erroneous behavior.

  Python, like many programming languages, provides mechanisms to avoid these access issues:

  

- **Global Interpreter Lock (GIL)**.

- **Locks, Locks, and Semaphores**.

- **Atomic Operations**: Certain operations in Python, like reading or replacing a variable, are atomic and thus thread-safe.

- **Using Thread-Safe Data Structures** like queue. Queue.

- **Design Considerations**: Sometimes, avoiding a shared state altogether or designing the system so that each thread operates on separate data can be a good strategy to prevent race conditions.



**什么是竞态条件？**

在Python中，当一个变量被两个不同的回调（或线程）同时访问时，可能会引发并发问题，通常称为竞态条件。 竞态条件发生在多个线程或进程同时访问共享数据，并且至少有一个线程对数据进行修改时。 这可能导致不可预测和错误的行为。

Python，像许多编程语言一样，提供了机制来避免这些访问问题：

md 就是同步，详情查看realtime systems 妈的

1. **全局解释器锁（Global Interpreter Lock，GIL）：**
   - GIL是Python中的一种机制，确保在同一时间只有一个线程可以执行Python字节码。这有助于防止多个线程同时修改共享数据。
2. **锁、互斥锁和信号量：**
   - 通过使用锁、互斥锁和信号量等同步机制，可以确保在某个线程正在修改共享数据时，其他线程不能同时访问。
3. **原子操作：**
   - Python中的某些操作，如读取或替换变量，是原子操作，因此是线程安全的。
4. **使用线程安全的数据结构，如队列（Queue）：**
   - 使用具有内置同步机制的数据结构，如队列，可以确保在多线程环境中的安全操作。
5. **设计考虑：**
   - 有时，避免共享状态或设计系统，使每个线程在独立的数据上操作，可以是预防竞态条件的有效策略。

总的来说，竞态条件是在多线程或多进程环境中可能发生的问题，通过使用锁、原子操作、线程安全的数据结构以及合适的设计策略，可以减轻或避免这些问题。 Python提供了多种机制来处理竞态条件，确保并发访问共享数据时的可靠性和一致性。

- See it in action.
- Add code to the script `face_searcher_multithreading_callbackgroups.py` to use Callback Groups.
- To solve the issue, use `Reentrantcallbacks` or `MutualyExclusiveCallbacks`.
- What would be the difference?

#### `ReentrantCallbackGroup` Solution

- Use **two threads**.
- Create only **ONE Callback Group** of type `ReentrantCallbackGroup`.
- Here, both callbacks will be executed *regardless of whether the other one in the group is executed.*
- You will need to add these modifications in Python to use a `Callback Group`.
- Topic subscribers, timers, service servers, action servers. They all have an argument where you can set the **callback group** their callbacks will be in.
- If nothing is specified, their callbacks will be assigned to the default `MutuallyExclusiveCallbackGroup` from the node.

```python
from rclpy.callback_groups import ReentrantCallbackGroup
...
self.reentrant_group_1 = ReentrantCallbackGroup()
...
self.subscription = self.create_subscription(
            Image,
            self._image_topic_name,
            self.listener_callback,
            10,
            callback_group=self.reentrant_group_1)

self.srv = self.create_service(Trigger, 'detect_faces', self.detect_faces_callback, callback_group=self.reentrant_group_1)
```

- The code should look something like this:

 **face_searcher_multithreading_callbackgroups.py [REENTRAND MOD]**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory
import os
from geometry_msgs.msg import Twist  # Import for velocity commands
from rclpy.duration import Duration
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class FaceDetectorNode(Node):
    def __init__(self, image_topic_name = "/deepmind_robot1/deepmind_robot1_camera/image_raw"):
        super().__init__('face_detector_node')


        self.reentrant_group_1 = ReentrantCallbackGroup()

        self._image_topic_name = image_topic_name
        self.subscription = self.create_subscription(
            Image,
            self._image_topic_name,
            self.listener_callback,
            10,
            callback_group=self.reentrant_group_1)
        self.subscription  # prevent unused variable warning
        self.srv = self.create_service(Trigger, 'detect_faces', self.detect_faces_callback, callback_group=self.reentrant_group_1)
        self.bridge = CvBridge()
        self.last_image = None

        # Publisher for cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        package_name = 'basics_ros2_multithreading'  # Replace with your package name
        package_path = get_package_share_directory(package_name)
        haar_cascade_file_path = os.path.join(package_path,"scripts/haarcascade_frontalface.xml")
        print(f"Package Path: {haar_cascade_file_path}")


        self.face_cascade = cv2.CascadeClassifier(haar_cascade_file_path)

    def listener_callback(self, data):
        self.get_logger().info('MULTITHREADING REENTRANT Receiving video frame')
        self.last_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

    def detect_faces_callback(self, request, response):
        start_time = self.get_clock().now()
        duration_10_sec = Duration(seconds=10)
        

        face_found = False

        # We start moving the robot
        self.publish_velocity(linear=0.0, angular=-0.3)
        delta_time = self.get_clock().now() - start_time
        self.get_logger().info('Delta Time='+str(delta_time))

        while delta_time < duration_10_sec and not face_found:
            delta_time = self.get_clock().now() - start_time
            self.get_logger().info('Delta Time='+str(delta_time))

            if self.last_image is not None:
                self.get_logger().info("Image....")
                gray = cv2.cvtColor(self.last_image, cv2.COLOR_BGR2GRAY)
                self.get_logger().info("Faces Processing....")
                faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)
                
                if len(faces) > 0:
                    self.publish_velocity(linear=0.0, angular=0.0)
                    self.get_logger().info("Face Found....")
                    face_found = True
                    self.get_logger().info('face_found='+str(face_found))
                    for (x, y, w, h) in faces:
                        cv2.rectangle(self.last_image, (x, y), (x+w, y+h), (255, 0, 0), 2)
                    
                    cv2.imshow("Faces", self.last_image)
                    cv2.waitKey(0)

                    break  # Exit the loop as a face is found
            self.get_logger().info("Sleep....")
            time.sleep(0.1)
            self.get_logger().info("End Sleep....")

        # We stop the robot
        self.publish_velocity(linear=0.0, angular=0.0)
        self.get_logger().info("End...")
        response.success = face_found
        response.message = 'Processed latest image'
        return response
    
    def publish_velocity(self, linear, angular):
        # Create Twist message
        vel_msg = Twist()
        vel_msg.linear.x = linear
        vel_msg.angular.z = angular
        # Publish the message
        self.cmd_vel_publisher.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    face_detector_node = FaceDetectorNode()
    
    # Use MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(face_detector_node)
    
    try:
        executor.spin()
    finally:
        face_detector_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```





 **face_searcher_multithreading_callbackgroups.launch.py [REENTRAND MOD]**

```python
import launch
import launch_ros.actions
from tracetools_launch.action import Trace


def generate_launch_description():
    
    # Trace
    trace = Trace(
        session_name='face_searcher'
    )
    # Nodes
    face_searcher_multithreading_node = launch_ros.actions.Node(
        package='basics_ros2_multithreading',
        executable='face_searcher_multithreading_callbackgroups.py',
        arguments=[],
        output='screen',
    )

    return launch.LaunchDescription([
        trace,
        face_searcher_multithreading_node
    ])
```

- Build and execute

```
cd ~/ros2_ws/
source /home/simulations/ros2_sims_ws/install/setup.bash
colcon build --packages-select basics_ros2_multithreading
source install/setup.bash

# We remove the tracing folder in case we had executed the comand already
rm -rf ~/.ros/tracing/face_searcher/
ros2 launch basics_ros2_multithreading face_searcher_multithreading_callbackgroups.launch.py
```

- Call the service `/detect_faces` to start the **FACE DETECTION SEARCH**.

```
cd ~/ros2_ws/
source install/setup.bash
ros2 service call /detect_faces std_srvs/srv/Trigger "{}
"
```

- You should now see that the robot starts **moving** and **ALWAYS** stops when it detects the first face.
- This means that the images arriving at the service callback `detect_faces_callback` are being updated **WHILE** the `detect_faces_callback` is executing.

#### MutuallyExclusiveCallbackGroup Solution

- Use **two threads**.
- Create **TWO Callback Groups** of type `MutuallyExclusiveCallbackGroup`.
- This allows you to execute both callbacks simultaneously **BECAUSE THEY ARE IN DIFFERENT** mutually exclusive callback groups.
- You will need to add these modifications in Python to use a **Callback Group**.

```python
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
...
self.mutuallyexclusive_group_1 = MutuallyExclusiveCallbackGroup()
self.mutuallyexclusive_group_2 = MutuallyExclusiveCallbackGroup()
...
self.subscription = self.create_subscription(
            Image,
            self._image_topic_name,
            self.listener_callback,
            10,
            callback_group=self.mutuallyexclusive_group_1)

self.srv = self.create_service(Trigger, 'detect_faces', self.detect_faces_callback, callback_group=self.mutuallyexclusive_group_2)
```

 **face_searcher_multithreading.py [MUTUALLYEXCLUSIVE MOD]**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory
import os
from geometry_msgs.msg import Twist  # Import for velocity commands
from rclpy.duration import Duration
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class FaceDetectorNode(Node):
    def __init__(self, image_topic_name = "/deepmind_robot1/deepmind_robot1_camera/image_raw"):
        super().__init__('face_detector_node')


        self.mutuallyexclusive_group_1 = MutuallyExclusiveCallbackGroup()
        self.mutuallyexclusive_group_2 = MutuallyExclusiveCallbackGroup()

        self._image_topic_name = image_topic_name
        self.subscription = self.create_subscription(
            Image,
            self._image_topic_name,
            self.listener_callback,
            10,
            callback_group=self.mutuallyexclusive_group_1)
        self.subscription  # prevent unused variable warning
        self.srv = self.create_service(Trigger, 'detect_faces', self.detect_faces_callback, callback_group=self.mutuallyexclusive_group_2)
        self.bridge = CvBridge()
        self.last_image = None

        # Publisher for cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        package_name = 'basics_ros2_multithreading'  # Replace with your package name
        package_path = get_package_share_directory(package_name)
        haar_cascade_file_path = os.path.join(package_path,"scripts/haarcascade_frontalface.xml")
        print(f"Package Path: {haar_cascade_file_path}")


        self.face_cascade = cv2.CascadeClassifier(haar_cascade_file_path)

    def listener_callback(self, data):
        self.get_logger().info('MULTITHREADING MUTUALLYEXCLUSIVE Receiving video frame')
        self.last_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

    def detect_faces_callback(self, request, response):
        start_time = self.get_clock().now()
        duration_10_sec = Duration(seconds=10)
        

        face_found = False

        # We start moving the robot
        self.publish_velocity(linear=0.0, angular=-0.3)
        delta_time = self.get_clock().now() - start_time
        self.get_logger().info('Delta Time='+str(delta_time))

        while delta_time < duration_10_sec and not face_found:
            delta_time = self.get_clock().now() - start_time
            self.get_logger().info('Delta Time='+str(delta_time))

            if self.last_image is not None:
                self.get_logger().info("Image....")
                gray = cv2.cvtColor(self.last_image, cv2.COLOR_BGR2GRAY)
                self.get_logger().info("Faces Processing....")
                faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)
                
                if len(faces) > 0:
                    self.publish_velocity(linear=0.0, angular=0.0)
                    self.get_logger().info("Face Found....")
                    face_found = True
                    self.get_logger().info('face_found='+str(face_found))
                    for (x, y, w, h) in faces:
                        cv2.rectangle(self.last_image, (x, y), (x+w, y+h), (255, 0, 0), 2)
                    
                    cv2.imshow("Faces", self.last_image)
                    cv2.waitKey(0)

                    break  # Exit the loop as a face is found
            self.get_logger().info("Sleep....")
            time.sleep(0.1)
            self.get_logger().info("End Sleep....")

        # We stop the robot
        self.publish_velocity(linear=0.0, angular=0.0)
        self.get_logger().info("End...")
        response.success = face_found
        response.message = 'Processed latest image'
        return response
    
    def publish_velocity(self, linear, angular):
        # Create Twist message
        vel_msg = Twist()
        vel_msg.linear.x = linear
        vel_msg.angular.z = angular
        # Publish the message
        self.cmd_vel_publisher.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    face_detector_node = FaceDetectorNode()
    
    # Use MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(face_detector_node)
    
    try:
        executor.spin()
    finally:
        face_detector_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

- And the new launch file:

**face_searcher_multithreading_callbackgroups.launch.py [MUTUALLYEXCLUSIVE MOD]**

```python
import launch
import launch_ros.actions
from tracetools_launch.action import Trace


def generate_launch_description():
    
    # Trace
    trace = Trace(
        session_name='face_searcher'
    )
    # Nodes
    face_searcher_multithreading_node = launch_ros.actions.Node(
        package='basics_ros2_multithreading',
        executable='face_searcher_multithreading_callbackgroups.py',
        arguments=[],
        output='screen',
    )

    return launch.LaunchDescription([
        trace,
        face_searcher_multithreading_node
    ])
```

- Build and execute

```
cd ~/ros2_ws/
source /home/simulations/ros2_sims_ws/install/setup.bash
colcon build --packages-select basics_ros2_multithreading
source install/setup.bash

# We remove the tracing folder in case we had executed the comand already
rm -rf ~/.ros/tracing/face_searcher/
ros2 launch basics_ros2_multithreading face_searcher_multithreading_callbackgroups.launch.py
```

- Now, call the service `/detect_faces` to start the **FACE DETECTION SEARCH**.



```
cd ~/ros2_ws/
source install/setup.bash
ros2 service call /detect_faces std_srvs/srv/Trigger "{}
"
```



- Now, visualize the `callbacks timeline`.
- It is obvious now that both callbacks `detect_faces_callback` and `listener_callback` are being executed simultaneously.
- The timeline is the same.

```
cd ~/ros2_ws/
source /home/simulations/ros2_sims_ws/install/setup.bash
ros2 run ros2_callback_visualiser analise_trace_face_searcher_callbacks.py
```

#### `MutuallyExclusive` or `Reentrant`?

- It comes down to the preferences and needs of your node.
- If your callback functions do not share resources like sensor input or variables, use `ReentrantCallbackGroups`.
- If you want to limit access to shared resources, `MutuallyExclusiveCallbackGroups` are the way to go.



### 6.5  Multiple Nodes in One Executor

- Typically, you will have **ONE ROS2 node** per Python script.
- However, there are instances when you want to execute the same script in several nodes.
- The reasons **why we would run multiple nodes with a single executor** are:
  - **Resource efficiency**: Launching **a SINGLE EXECUTOR** versus **TWO or as many as the number of nodes** will dramatically improve the performance and resource economy because all the threads will be shared between the nodes.
  - **Lower Latency in Communication**: Intra-process communication between nodes of the **SAME executor** is **FASTER** than between nodes of **DIFFERENT EXECUTORS**.

- Review an example.
- In this example, you add a new node named `yellow_tshirt_node` to the executor.
- This node will have a new *service callback named* **detect_yellow_tshirt_callback**.
- This means that you are initializing an extra **THREAD** in a separate script.
- However, if you use the same executor, you use the **TWO threads you have** more efficiently.
- It also gives direct access to the `FaceDetectorNode` and all its data, especially the *image data*.
- This structure makes your system more efficient threads-wise, using one less.

- To add a node to an executor is as simple as the following|:

```python
face_detector_node = FaceDetectorNode()
yellow_tshirt_node = YellowTShirtNode(face_detector_node)

# Use MultiThreadedExecutor
executor = MultiThreadedExecutor(num_threads=2)
executor.add_node(face_detector_node)
executor.add_node(yellow_tshirt_node)

try:
    executor.spin()
finally:
    face_detector_node.destroy_node()
    yellow_tshirt_node.destroy_node()
    rclpy.shutdown()
```

- Note that this `yellow_tshirt_node` has as input the `face_detector_node`.
- This allows you to access the latest image from the image subscriber without having to declare its own subscriber, reducing the number of threads needed.

- Add the code:

 **yello_tshirt_detector.py **  

包括两个node：FaceDetectorNode   YellowTShirtNode

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory
import os
from geometry_msgs.msg import Twist  # Import for velocity commands
from rclpy.duration import Duration
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import numpy as np

class FaceDetectorNode(Node):
    def __init__(self, image_topic_name = "/deepmind_robot1/deepmind_robot1_camera/image_raw"):
        super().__init__('face_detector_node')


        self.reentrant_group_1 = ReentrantCallbackGroup()

        self._image_topic_name = image_topic_name
        self.subscription = self.create_subscription(
            Image,
            self._image_topic_name,
            self.listener_callback,
            10,
            callback_group=self.reentrant_group_1)
        self.subscription  # prevent unused variable warning
        self.srv = self.create_service(Trigger, 'detect_faces', self.detect_faces_callback, callback_group=self.reentrant_group_1)
        self.bridge = CvBridge()
        self.last_image = None

        # Publisher for cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        package_name = 'basics_ros2_multithreading'  # Replace with your package name
        package_path = get_package_share_directory(package_name)
        haar_cascade_file_path = os.path.join(package_path,"scripts/haarcascade_frontalface.xml")
        print(f"Package Path: {haar_cascade_file_path}")


        self.face_cascade = cv2.CascadeClassifier(haar_cascade_file_path)

    def listener_callback(self, data):
        self.get_logger().info('Yello TSHIRT Receiving video frame')
        self.last_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

    def detect_faces_callback(self, request, response):
        start_time = self.get_clock().now()
        duration_10_sec = Duration(seconds=10)
        

        face_found = False

        # We start moving the robot
        self.publish_velocity(linear=0.0, angular=-0.3)
        delta_time = self.get_clock().now() - start_time
        self.get_logger().info('Delta Time='+str(delta_time))

        while delta_time < duration_10_sec and not face_found:
            delta_time = self.get_clock().now() - start_time
            self.get_logger().info('Delta Time='+str(delta_time))

            if self.last_image is not None:
                self.get_logger().info("Image....")
                gray = cv2.cvtColor(self.last_image, cv2.COLOR_BGR2GRAY)
                self.get_logger().info("Faces Processing....")
                faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)
                
                if len(faces) > 0:
                    self.publish_velocity(linear=0.0, angular=0.0)
                    self.get_logger().info("Face Found....")
                    face_found = True
                    self.get_logger().info('face_found='+str(face_found))
                    for (x, y, w, h) in faces:
                        cv2.rectangle(self.last_image, (x, y), (x+w, y+h), (255, 0, 0), 2)
                    
                    cv2.imshow("Faces", self.last_image)
                    cv2.waitKey(0)

                    break  # Exit the loop as a face is found
            self.get_logger().info("Sleep....")
            time.sleep(0.1)
            self.get_logger().info("End Sleep....")

        # We stop the robot
        self.publish_velocity(linear=0.0, angular=0.0)
        self.get_logger().info("End...")
        response.success = face_found
        response.message = 'Processed latest image'
        return response
    
    def publish_velocity(self, linear, angular):
        # Create Twist message
        vel_msg = Twist()
        vel_msg.linear.x = linear
        vel_msg.angular.z = angular
        # Publish the message
        self.cmd_vel_publisher.publish(vel_msg)

    def get_latest_image(self):
        return self.last_image
    
    def get_face_cascade(self):
        return self.face_cascade



class YellowTShirtNode(Node):
    def __init__(self, face_detect_node):
        super().__init__('yellow_tshirt_node')

        self._face_detect_node = face_detect_node

        self.service = self.create_service(Trigger, 'yellow_tshirt', self.detect_yellow_tshirt_callback)
        self.bridge = CvBridge()

    def detect_yellow_tshirt_callback(self, request, response):
        cv_image = self.get_image()
        response.success, response.message = self.detect_yellow_tshirt(cv_image)
        return response
    
    def get_image(self):
        return self._face_detect_node.get_latest_image()

    def detect_yellow_tshirt(self, cv_image):
        # Detect bodies
        # Convert the image to HSV color space
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define the lower and upper bounds for the yellow color in HSV
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])

        # Create a mask to threshold the image to detect yellow
        mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

        # Publish the binary mask as an image
        cv2.imshow("BODY", mask)
        cv2.waitKey(0)

        return True, "Yellow"  # No smile detected


def main(args=None):
    rclpy.init(args=args)
    face_detector_node = FaceDetectorNode()
    yellow_tshirt_node = YellowTShirtNode(face_detector_node)
    
    # Use MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(face_detector_node)
    executor.add_node(yellow_tshirt_node)
    
    try:
        executor.spin()
    finally:
        face_detector_node.destroy_node()
        yellow_tshirt_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**yello_tshirt_detector.launch.py**

```python
import launch
import launch_ros.actions
from tracetools_launch.action import Trace


def generate_launch_description():
    
    # Trace
    trace = Trace(
        session_name='yello_tshirt_detector'
    )
    # Nodes
    face_searcher_multithreading_node = launch_ros.actions.Node(
        package='basics_ros2_multithreading',
        executable='yello_tshirt_detector.py',
        arguments=[],
        output='screen',
    )

    return launch.LaunchDescription([
        trace,
        face_searcher_multithreading_node
    ])
```

- Build and execute

```
cd ~/ros2_ws/
source /home/simulations/ros2_sims_ws/install/setup.bash
colcon build --packages-select basics_ros2_multithreading
source install/setup.bash

# We remove the tracing folder in case we had executed the comand already
rm -rf ~/.ros/tracing/yello_tshirt_detector/
ros2 launch basics_ros2_multithreading  yello_tshirt_detector.launch.py
```

- Now, call the service `/detect_faces` to start the **FACE DETECTION SEARCH**.

```
cd ~/ros2_ws/
source install/setup.bash
ros2 service call /detect_faces std_srvs/srv/Trigger "{}
"
```

- **AND NOW**, call the service `/yellow_tshirt` to start the **Yellow t-shirt detection**.

```
cd ~/ros2_ws/
source install/setup.bash
ros2 service call /yellow_tshirt std_srvs/srv/Trigger "{}
"
```



- Now, visualize the **callbacks timeline**.

```
cd ~/ros2_ws/
source /home/simulations/ros2_sims_ws/install/setup.bash
ros2 run ros2_callback_visualiser analise_trace_yellow_tshirt_callbacks.py
```

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_ROS2/images/multinodeexecutor.png)

- As you can see, **TWO THREADS** were enough for this functionality.
- In any instance of your execution, the system only executes **TWO CALLBACKS** simultaneously.
- However, **what if you call both services simultaneously?**
- Because you only have **TWO threads**... one of the three available callbacks will not be executed.
- The one that finishes faster is the `listener_callback` so that one will eventually get paused while both service callbacks are executed.
- It has nothing to do with priority. Once you have the callback in the callback queue, service callbacks tend to last longer.
- You can see represented in color circles each of the callbacks being executed at that moment in time.
- 在任何执行实例中，系统只会同时执行两个回调。然而，如果同时调用了三个服务，由于只有两个线程，三个可用回调中的一个将不会被执行。
- 执行速度更快的回调是`listener_callback`，因此在执行两个服务回调时，`listener_callback`将最终被暂停。这与回调的优先级无关，而是由于一旦将回调放入回调队列，服务回调通常需要更长的时间。
- 文中通过使用颜色圆圈表示每个回调在某个时间点被执行，说明了同时执行两个服务回调和一个订阅回调的情况。

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_ROS2/images/multinodeshow2threads.png)

### 6.6  Summary

- You have learned how to add **MULTITHREADING** to your ROS2 nodes.
- You have learned that ROS2 nodes have one thread and one MutuallyExclusive Callback group by default. All the node's callbacks will be placed in that default callback group.
- To execute multiple callbacks using multithreading, use `ReentrantCallbacksGroups` or `MutuallyExclusiveCallbackGroups`.
- Both solutions give the same result.
- You also learned how to add multiple nodes in the same executor and why this could be beneficial.





## 7  Understanding ROS2 Actions

**What will you learn with this unit?**

- Understand an Action
- How to call an Action Server
- How to write an Action Server
- How to create your own Action Interface
- How to use your Action Interface

### 7.1 什么是 Action?



Action是ROS 2中的一种高级通信机制，专门设计用于处理需要反馈和可抢占性的长时间运行任务。通过Action Server和Action Client的配合使用，节点可以异步地执行任务，并且在任务执行过程中，客户端可以接收到服务器的实时反馈，也可以在需要时取消正在执行的任务。



Actions与Services在ROS 2中扮演类似的角色，都是基于客户端-服务器模型，用于实现节点间的功能调用。然而，Actions与Services之间存在两个关键的区别：

1. **可抢占性（Preemptable）**：Actions的一个主要特点是它们可以在执行过程中被取消或抢占。这意味着，如果客户端发起了一个Action，并且在该Action执行的任何时刻，客户端都可以决定取消这个Action。这与Services不同，后者一旦被调用，就会执行到完成，无法中途取消。
2. **反馈（Feedback）**：与Services相比，Actions提供执行过程中的反馈功能。当一个Action被执行时，服务器可以定期向客户端发送进度更新或其他类型的反馈信息，让客户端了解Action执行的当前状态。

在ROS 2中实现Action通信机制的工作流程大致如下：

- **客户端（Action Client）**发送一个目标（goal）给服务器（Action Server），这标志着Action的开始。
- **服务器（Action Server）**在Action执行过程中，可以向客户端（Action Client）发送反馈信息。
- 一旦Action完成，服务器将执行结果（response）返回给客户端。

在技术实现上，Actions利用Services来处理**目标（goal）**和**结果（result）**，同时使用Topics来处理反馈（feedback）。这种设计使Actions既能够支持复杂的、需要反馈的长期任务，也能够在必要时进行取消或抢占。

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_ROS2/images/ros2-actions.gif)

In summarizing, the workflow goes like this:

1. The **Client sends a goal** to the Server. This will trigger the "start" of the Action.
2. The **Server sends feedback** to the Client while the Action is taking place.
3. Once the Action finishes, the **Server returns a response** to the Client.





```
source /home/simulations/ros2_sims_ws/install/setup.bash
```

```
ros2 launch turtlebot3_as action_server.launch.py
```

To find which actions are available on a robot, use the command ros2 action list.

```
source /home/simulations/ros2_sims_ws/install/setup.bash
ros2 action list
```

One of the actions in the list should be the `/turtlebot3_as`

You can also get data from a specific Action with the following command:

```
ros2 action info /turtlebot3_as
```

#### 查看action的详细信息

```
Action: /turtlebot3_as
Action clients: 0
Action servers: 1
    /t3_action_server
```

Also, if you add the suffix `-t` to the command above, you will get data from the action interface used:

```
ros2 action info /turtlebot3_as -t
```

```
Action: /turtlebot3_as
Action clients: 0
Action servers: 1
    /t3_action_server [t3_action_msg/action/Move]
```

输出显示`/turtlebot3_as` action使用了`t3_action_msg/action/Move`作为其接口。

- **Action**: 显示你查询的action的名称，这里是`/turtlebot3_as`。
- **Action clients**: 显示当前连接到该action服务器的客户端数量。在此例中，数量为0，意味着没有客户端当前正在使用这个action。
- **Action servers**: 显示提供该action功能的服务器数量。在此例中，数量为1，表明有一个服务器提供了`/turtlebot3_as` action的服务。
- **/t3_action_server**: 这是提供`/turtlebot3_as` action服务的节点的名称。
- **[t3_action_msg/action/Move]**: 这表明`/turtlebot3_as` action使用了`Move`接口，而这个接口定义在`t3_action_msg/action`包中。这是action接口的名称，描述了action的结构，包括它需要的目标(goal)、提供的反馈(feedback)以及最终结果(result)。

```
<pkg_name>/action/<interface_name>
```

In a different Action Server, the name of the package and the interface may be different. All Action Interfaces are defined inside a folder called **`action`**.

You can also get more data about this interface with the following command:

```
ros2 interface show t3_action_msg/action/Move
```

```
int32 secs
---
string status
---
string feedback
```

可以看到，interface定义了发送的goal信息的类型和变量名，secs

#### 发送goal，查看feedback

All right! Now that you have gathered some data about the Action Server, call it!

- To call an Action Server, you can use the command **`ros2 action send_goal`**.
- The structure of the command is the following:

```
ros2 action send_goal <action_name> <action_type> <values>
```

Now that you have all the data about the Action Server, you can complete the command:

根据interface的定义，给发送的goal进行赋值secs: 5

```
ros2 action send_goal /turtlebot3_as t3_action_msg/action/Move "{secs: 5}"


Waiting for an action server to become available...
Sending goal:
     secs: 5

Goal accepted with ID: f4eb62b707b74881b923a44a7d3d60f9

Result:
    status: Finished action server. Robot moved during 5 seconds

Goal finished with status: SUCCEEDED
```

可是我现在只看到了goal，没有看到feedback，怎么查看feedback呢？

```
ros2 action send_goal -f /turtlebot3_as t3_action_msg/action/Move "{secs: 5}"
```

Note the `-f` argument added to the command, which is the short flag form for `--feedback`. You can use either as you wish.

```
Waiting for an action server to become available...
Sending goal:
     secs: 5

Goal accepted with ID: db711620eb284b9f8d69d54fc49b4155

Feedback:
    feedback: Moving to the left left left...

Feedback:
    feedback: Moving to the left left left...

Feedback:
    feedback: Moving to the left left left...

Feedback:
    feedback: Moving to the left left left...

Feedback:
    feedback: Moving to the left left left...

Result:
    status: Finished action server. Robot moved during 5 seconds

Goal finished with status: SUCCEEDED
```

### 7.2  Calling an Action Server

`turtlebot3_as` Action Server是一个可以被调用的Action，专门用于控制TurtleBot3机器人。当这个Action被调用时，它会使TurtleBot3机器人向前移动，移动的时间长度由调用消息中指定的参数决定。

调用一个Action Server本质上意味着向它发送一个目标（goal）。这一过程与Topics（主题）和Services（服务）相似，都是通过消息传递机制来实现的。不同类型的ROS 2通信机制使用的消息结构不同：

- **Topic的消息**只包含单一部分，即Topic提供的信息。
- **Service的消息**包含两部分：请求（request）和响应（response）。
- **Action Server的消息**则更为复杂，分为三个部分：目标（goal）、结果（result）和反馈（feedback）。

所有这些Action消息的定义都位于它们所属包的Action目录中。

如果你查看`t3_action_msg`包，你会发现其中包含一个名为`Action`的目录。在`Action`目录内，有一个名为`Move.action`的文件。这个文件指定了Action使用的消息类型。具体来说：

- **目标（Goal）**：调用Action时要传递给Action Server的信息，例如在`turtlebot3_as`的例子中，目标可能是机器人应该前进的秒数。
- **结果（Result）**：Action完成后，Action Server返回给Action Client的信息，比如操作是否成功完成，或者实际移动了多久。
- **反馈（Feedback）**：在Action执行过程中，Action Server可以不断向Action Client提供的实时信息，例如当前的进度或状态。

这种结构允许ROS 2的Action不仅能够执行长时间运行的任务，还能够在执行过程中提供反馈，并最终返回一个结果，这为复杂的机器人任务提供了极大的灵活性和控制能力。

If you check the **`t3_action_msg`** package, you will see that it contains a directory called Action. Inside that Action directory, there is a file called `Move.action`. That is the file specifying the type of message the Action uses.

In this case, the **`t3_action_msg`** package is installed in the system. You can check it out by running the following command:

```
cd /home/simulations/ros2_sims_ws/src/t3_foxy/t3_action_msg/
```



Type the following command in a Terminal to see the message structure:

```
ros2 interface show t3_action_msg/action/Move
```

You can see in the output that the message is composed of three parts:

- **`goal`**: Consists of a variable called **`secs`** of type **`int32`**.
- **`result`**: Consists of a variable called **`status`**, which is of type **`string`**.
- **`feedback`**: Consists of a variable called **`feedback`** of type **`string`**.

```
# goal
int32 secs # the number of seconds the robot will move forward
---
# result
string status # a string indicating the final status when the Action ends
---
# feedback
string feedback # a string that indicates the current status of the robot
```



#### 7.2.1  Actions provide feedback

调用Action Server的一个关键特点是它不会中断你的线程。这意味着当一个Action Server被调用执行某个任务时，发起调用的程序（**Server**）可以继续执行其他操作，而不需要等待Action完成。为了让客户端能够了解到Action执行的状态，ROS 2引入了一种特殊的消息类型——**反馈（feedback）**。

**feedback**是Action Server在Action执行过程中偶尔生成的消息，用于向客户端通报当前Action的执行情况。这允许客户端获得有关所请求Action执行状态的实时信息，即使客户端自身的执行线程没有被阻塞。

1. **非阻塞调用**：Action Servers的设计允许调用操作（如命令机器人执行任务）时，不会阻断调用者的线程。这对于需要同时执行多项任务的复杂机器人应用尤其重要。
2. **反馈机制**：通过提供执行过程中的反馈消息，Action Servers使客户端能够实时监控Action的执行状态。这种反馈对于需要长时间执行或需要密切监控进度的任务尤其有用。
3. **实时状态更新**：反馈消息为客户端提供了一种获取Action执行进度的手段，即便该Action尚未完成，也能让客户端了解当前的执行情况。

#### 7.2.2  How to call an Action Server

The way you call an Action Server is by implementing an **Action Client**.

In Example 7.1, you created an **action client** using the command-line tools, with the command **ros2 action send_goal**. 

创建一个 **action client** ，在命令行控制它发送Goal

Instead, you will usually have to implement an **action client** by creating a program.

The following is a self-explanatory example of how to implement an **action client** that calls the **turtlebot3_as** Action Server and makes it move forward for five seconds.

##### 1 创建packagemy_action_client

**First,** create a new package where you will place your **action client** code.

```
cd ~/ros2_ws/src
ros2 pkg create my_action_client --build-type ament_python --dependencies rclpy rclpy.action t3_action_msg
```

##### 2 **action_client.py**：

Now, inside the **`my_action_client`** folder, create a new **`Python`** file named **`action_client.py`**. Then, you can paste the code below into the script.

my_action_client/**action_client.py**：

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from t3_action_msg.action import Move


class MyActionClient(Node):

    def __init__(self):
        super().__init__('my_action_client')
        self._action_client = ActionClient(self, Move, 'turtlebot3_as')

    def send_goal(self, seconds):
        goal_msg = Move.Goal()
        goal_msg.secs = seconds

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.status))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            'Received feedback: {0}'.format(feedback.feedback))


def main(args=None):
    rclpy.init(args=args)

    action_client = MyActionClient()

    action_client.send_goal(5)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
```

##### 3 **Modify** the `setup.py`:

```python
from setuptools import setup
import os
from glob import glob

package_name = 'my_action_client'

setup(
    name='my_action_client',
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
            'example62 = my_action_client.action_client:main'
        ],
    },
)
```

##### 4 **Create a launch file**

**`example62_launch_file.launch.py`**

```
cd ~/ros2_ws/src/my_action_client
mkdir launch
cd ~/ros2_ws/src/my_action_client/launch
touch example62_launch_file.launch.py
chmod +x example62_launch_file.launch.py
```

##### 5 **Compile** your package

```
cd ~/ros2_ws
colcon build --packages-select my_action_client
source ~/ros2_ws/install/setup.bash
```

##### 6 执行文件

**Then,** launch the Action Server node from Example 7.1 on your Terminal #1.

```
ros2 launch turtlebot3_as action_server.launch.py
```

**Finally,** launch the Action Client node on your Terminal #2.

```
source ~/ros2_ws/install/setup.bash
ros2 launch my_action_client example62_launch_file.launch.py
```

#### 7.2.3  Action Client code explanation

代码详解：

````python
# 导入必要的ROS 2 Python客户端库
import rclpy
from rclpy.action import ActionClient  # 用于创建和管理action client
from rclpy.node import Node  # Node类，所有ROS 2节点的基类

# 导入自定义的Move action类型，这是从你自定义的action定义中自动生成的
from t3_action_msg.action import Move

# 定义MyActionClient类，继承自Node
class MyActionClient(Node):

    def __init__(self):
        # 初始化节点，节点名为'my_action_client'
        super().__init__('my_action_client')
        # 创建ActionClient对象，用于与名为'turtlebot3_as'的action server通信
        # 参数解释：
        # self: 表示这个ActionClient关联的节点
        # Move: Action类型
        # 'turtlebot3_as': Action的名称，这需要与action server端匹配
        self._action_client = ActionClient(self, Move, 'turtlebot3_as')

    def send_goal(self, seconds):
        # 创建一个Move.Goal消息对象
        goal_msg = Move.Goal()
        goal_msg.secs = seconds  # 设置目标消息的secs字段，表示需要执行动作的秒数

        # 等待action server变得可用
        self._action_client.wait_for_server()

        # 异步发送目标到action server，同时注册一个反馈回调函数
        # 参数解释：
        # goal_msg: 包含目标信息的消息对象
        # feedback_callback: 当接收到反馈时调用的回调函数
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        
        # 为send_goal_async返回的future对象添加一个完成回调
        # 当目标请求被处理（接受或拒绝）时调用
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    # 目标响应回调函数
    def goal_response_callback(self, future):
        goal_handle = future.result()  # 获取future的结果，即goal handle
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')  # 目标被拒绝
            return

        self.get_logger().info('Goal accepted :)')  # 目标被接受

        # 请求获取action执行的结果
        self._get_result_future = goal_handle.get_result_async()
        # 为获取结果的future添加完成回调
        self._get_result_future.add_done_callback(self.get_result_callback)

    # 获取结果的回调函数
    def get_result_callback(self, future):
        result = future.result().result  # 获取action的执行结果
        self.get_logger().info('Result: {0}'.format(result.status))  # 打印结果状态
        rclpy.shutdown()  # 结束节点运行

    # 反馈回调函数
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback  # 获取反馈信息
        self.get_logger().info('Received feedback: {0}'.format(feedback.feedback))  # 打印反馈信息

# main函数定义
def main(args=None):
    rclpy.init(args=args)  # 初始化ROS 2

    action_client = MyActionClient()  # 创建MyActionClient实例

    action_client.send_goal(5)  # 发送目标给action server，这里的sec是5秒

    rclpy.spin(action_client)  # 保持节点运行，直到收到退出信号

# Python入口点
if __name__ == '__main__':
    main()


````

In this section, you will break down the code you developed in Section 7.2.2, where I showed you how to write an Action Client. Begin with the first section, where you will import the necessary libraries.

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from t3_action_msg.action import Move
```

As you may have noticed, in the first three lines, you do nothing more than import the ROS2 Python client libraries to work with Actions (`rclpy.action`) and nodes (`rclpy.node`). You can also see that this is where you will import the interfaces you work with, in this case, (`t3_action_msg.action`).

Now jump to the `main` function.

```python
def main(args=None):
    rclpy.init(args=args)

    action_client = MyActionClient()

    action_client.send_goal(5)

    rclpy.spin(action_client)
```

Here you are creating an instance of the **`MyActionClient()`** class and calling its `send_goal()` method, passing it five seconds as a parameter. (In a moment, you will analyze the contents of this method). Then, finally, spin the node with `rclpy.spin()` so that the callback functions are properly executed.

Now continue analyzing the constructor of the class **`MyActionClient()`**:

```python
def __init__(self):
        super().__init__('my_action_client')
        self._action_client = ActionClient(self, Move, 'turtlebot3_as')
```

In the constructor of the class, initialize a ROS2 node named `my_action_client`. Also, and very important, create an `ActionClient` object to which you pass three arguments:

1. The ROS2 node that contains the Action Client: in this case, **`self`**.
2. The type of the Action: **`Move`** (related to the `t3_action_msg` interface of type Action).
3. The Action name: **`turtlebot3_as`**.

Now continue analyzing the `send_goal()` method:

```python
def send_goal(self, seconds):
        goal_msg = Move.Goal()
        goal_msg.secs = seconds

        self._action_client.wait_for_server()
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
```

Start by creating a **`Goal()`** object of the **`Move`** action type. Then, access the **`secs`** variable of the Action goal and assign it the value of `seconds` (which is five in this example).

```python
goal_msg = Move.Goal()
goal_msg.secs = seconds
```

\- Notes -

Remember the structure of the action type **`Move`** that you checked earlier:

```python
int32 secs
---
string status
---
string feedback
```

\- End of Notes -

Next, wait for the Action Server to be up and running:

```python
self._action_client.wait_for_server()
```

And send the goal to the Server using the **`send_goal_async`** method:

```python
self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
```

You need to provide two arguments for this method:

- A goal message, in this case, **`goal_msg`**
- A callback function for the feedback, in this case, **`self.feedback_callback`**
- **`send_goal_async()`方法**：这是一个非阻塞（异步）方式发送目标（goal）到action服务器的方法。这意味着当你调用这个方法时，它会立即返回，而不会等待action服务器处理完这个目标。这使得你的程序可以继续执行其他任务，而不是在等待服务器响应时停滞不前。
- **返回一个future到goal handle**：这个方法不会直接返回一个goal handle（一个代表特定action目标的对象），而是返回一个`future`对象。在编程中，`future`是一种模式，用于表示一个可能还没有完成的操作的结果。在这个上下文中，`future`对象代表了goal handle的最终结果，这个结果在未来的某个时刻会变得可用。
- **这个future goal handle将在服务器处理完目标时完成**：这意味着`future`对象将在action服务器接受或拒绝你发送的目标时“完成”。换句话说，一旦服务器决定了它会处理这个目标（接受）或者不会处理（拒绝），`future`对象就会包含这个最终结果（即goal handle），此时你可以查询这个`future`对象来知道目标是否被接受，并据此进行下一步的操作。
- **处理目标意味着被接受或拒绝**：在这里，“处理”一个目标包括了两种可能性：目标被接受或被拒绝。如果目标被接受，action服务器会开始执行相应的操作，并最终返回一个结果；如果目标被拒绝，那么不会有进一步的操作执行，客户端应当根据这一反馈决定是否需要采取其他行动。

This **`send_goal_async()`** method returns a future to a goal handle. **This future goal handle will be completed when the Server has processed the goal. (This means it has been accepted or rejected by the Server)**. So, you must assign a callback method to be triggered when the future is completed (the goal has been accepted or rejected). In this case, this method is **`self.goal_response_callback`**:

```python
self._send_goal_future.add_done_callback(self.goal_response_callback)
```

Now have a look at this **`self.goal_response_callback`** method:

```python
def goal_response_callback(self, future):
    goal_handle = future.result()
    if not goal_handle.accepted:
        self.get_logger().info('Goal rejected :(')
        return

    self.get_logger().info('Goal accepted :)')

    self._get_result_future = goal_handle.get_result_async()
    self._get_result_future.add_done_callback(self.get_result_callback)
```

So, this method will be triggered when the goal has been processed. First, check whether the Server has accepted the goal:

```python
goal_handle = future.result()
        if not goal_handle.accepted:
```

Print a message if it has been rejected. If it has been accepted, ask for the result using the **`get_result_async()`** method:

```python
self._get_result_future = goal_handle.get_result_async()
```

Similar to sending the goal, this method will return a future that will be completed when the result is ready. So, you must also assign a callback method to be triggered when this future is completed (the result is ready). In this case, this method is **`self.get_result_callback`**:

```python
self._get_result_future.add_done_callback(self.get_result_callback)
```

Now, have a look at this **`self.get_result_callback`** method:

```python
def get_result_callback(self, future):
    result = future.result().result
    self.get_logger().info('Result: {0}'.format(result.status))
    rclpy.shutdown()
```

This method is very simple. You get the result (**`future.result().result`**), print, and then shut down the node for a clean exit with `rclpy.shutdown()`.

Finally, you have the feedback callback method:

```python
def feedback_callback(self, feedback_msg):
    feedback = feedback_msg.feedback
    self.get_logger().info(
        'Received feedback: {0}'.format(feedback.feedback))
```

Here you get the **`feedback`** string from the **`feedback_msg`** and print it to the screen.

And that is it! You have reviewed the most important parts of the Action Client node code.



### 7.3  Writing an Action Server

#### 1 创建package`my_action_server`

```
cd ~/ros2_ws/src
ros2 pkg create my_action_server --build-type ament_python --dependencies rclpy rclpy.action t3_action_msg
```

#### 2 action_server.py

Now, inside the **`my_action_server`** folder, create a new Python file named **`action_server.py`**. You can paste the code below into the script:

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from t3_action_msg.action import Move

from geometry_msgs.msg import Twist
import time
class MyActionServer(Node):

    def __init__(self):
        super().__init__('my_action_server')
        self._action_server = ActionServer(self, Move, 'turtlebot3_as_2',self.execute_callback) 
        self.cmd = Twist()
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
    
    
    
    def execute_callback(self, goal_handle):
        
        self.get_logger().info('Executing goal...')

        feedback_msg = Move.Feedback()
        feedback_msg.feedback = "Moving to the left left left..."

        for i in range(1, goal_handle.request.secs):
            
            self.get_logger().info('Feedback: {0} '.format(feedback_msg.feedback))

            goal_handle.publish_feedback(feedback_msg)
            self.cmd.linear.x = 0.3
            self.cmd.angular.z =0.3
            
            self.publisher_.publish(self.cmd)
            time.sleep(1)

        goal_handle.succeed()

        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
            
        self.publisher_.publish(self.cmd)
        result = Move.Result()
        result.status = "Finished action server. Robot moved during 5 seconds"
        self.get_logger().info('Result: {0}'.format(result.status))
        return result

def main(args=None):
    rclpy.init(args=args)

    my_action_server = MyActionServer()

    rclpy.spin(my_action_server)


if __name__ == '__main__':
    main()
```

#### 3 修改setup.py

```python
from setuptools import setup
import os
from glob import glob

package_name = 'my_action_server'

setup(
    name='my_action_server',
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
            'example63 = my_action_server.action_server:main'
        ],
    },
)
```

#### 4 **Create a launch file**

**`example63_launch_file.launch.py`**

```
cd ~/ros2_ws/src/my_action_server
mkdir launch
cd ~/ros2_ws/src/my_action_server/launch
touch example63_launch_file.launch.py
chmod +x example63_launch_file.launch.py
```

**example63_launch_file.launch.py**

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_action_server',
            executable='example63',
            output='screen'),
    ])
```

注意，launch中的executable应当与setup.py-entry_points-console_scripts的名字相同。

#### 5 **Compile** your **package**

```shell
cd ~/ros2_ws
colcon build --packages-select my_action_server
source ~/ros2_ws/install/setup.bash
```

#### 6 launch the **`action_server`**

```
ros2 launch my_action_server example63_launch_file.launch.py
```

Before running the Client node, make sure to update the name of the Action to **`turtlebot3_as_2`**:

```
self._action_client = ActionClient(self, Move, 'turtlebot3_as_2')
```

You will need to build again the `my_action_client` package for the change to be applied.

#### 7 execute your Client node

```shell
cd ~/ros2_ws
colcon build --packages-select my_action_client
source ~/ros2_ws/install/setup.bash
ros2 launch my_action_client example62_launch_file.launch.py
```

#### 8 Action Server code explanation

- **Action的名称**是指在ROS系统中，客户端（Action Client）和服务器（Action Server）通过一个特定的名字进行通信的标识符。这个名称用于标识特定的Action服务，允许客户端知道它应该将目标（goal）发送到哪个服务器。在你的例子中，`'turtlebot3_as_2'`是Action的名称，用于客户端和服务器之间的匹配和通信。这个名字是自定义的。

- **ActionServer的名称**通常指的是创建ActionServer时给它的ROS节点的名称。在ROS中，每个节点都应该有一个唯一的名称，这样系统可以区分不同的节点。在提供的代码中，通过`super().__init__('my_action_server')`初始化的节点名称为`'my_action_server'`。这个名称是内部用来识别ROS节点的，而不是用于Action通信的标识符。

  不同的Action Server不能使用相同的Action名称进行通信。在ROS中，每个Action名称都对应一个唯一的Action Server，用于处理特定类型的Action请求。这种设计确保了客户端发送的目标（goal）能够准确地被送达到预期的服务器，并由该服务器处理。

```python
# 导入必要的ROS 2和消息类型的Python库
import rclpy
from rclpy.action import ActionServer  # 用于创建action server
from rclpy.node import Node  # 所有ROS 2节点的基类
from t3_action_msg.action import Move  # 导入自定义的Move action类型
from geometry_msgs.msg import Twist  # Twist消息类型用于控制机器人的线速度和角速度
import time  # 导入time库用于暂停

# 定义MyActionServer类，继承自Node
class MyActionServer(Node):

    def __init__(self):
        # 初始化节点，命名为'my_action_server'
        super().__init__('my_action_server')
        # 创建ActionServer对象
        # 参数解释：
        # self: 表示这个ActionServer关联的节点
        # Move: Action类型
        # 'turtlebot3_as_2': Action的名称，客户端将通过此名称发送请求
        # self.execute_callback: 当接收到目标请求时执行的回调函数
        self._action_server = ActionServer(self, Move, 'turtlebot3_as_2', self.execute_callback)
        
        # 初始化用于发送移动命令的Twist消息
        self.cmd = Twist()
        # 创建一个发布者，用于发布Twist消息控制机器人
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
    
    # 执行目标的回调函数
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        # 初始化反馈消息
        feedback_msg = Move.Feedback()
        feedback_msg.feedback = "Moving to the left left left..."
        
        # 根据目标请求的时长执行动作
        for i in range(1, goal_handle.request.secs):
            # 发送反馈信息
            self.get_logger().info('Feedback: {0} '.format(feedback_msg.feedback))
            goal_handle.publish_feedback(feedback_msg)
            
            # 设置机器人的移动命令
            self.cmd.linear.x = 0.3  # 前进速度
            self.cmd.angular.z = 0.3  # 转向速度
            
            # 发布移动命令
            self.publisher_.publish(self.cmd)
            time.sleep(1)  # 暂停一秒钟

        # 执行完成后停止机器人
        goal_handle.succeed()
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)
        
        # 设置并返回执行结果
        result = Move.Result()
        result.status = "Finished action server. Robot moved during 5 seconds"
        self.get_logger().info('Result: {0}'.format(result.status))
        return result

# 主函数，初始化ROS 2并运行action server
def main(args=None):
    rclpy.init(args=args)  # 初始化ROS 2
    my_action_server = MyActionServer()  # 创建MyActionServer实例
    rclpy.spin(my_action_server)  # 保持节点运行，直到收到退出信号

if __name__ == '__main__':
    main()

```



**导入必要的库**

```python
pythonCopy code解释import rclpy  # 导入ROS 2 Python客户端库
from rclpy.action import ActionServer  # 允许你创建一个Action Server节点
from rclpy.node import Node  # 基础节点类

from t3_action_msg.action import Move  # 导入自定义的Move action类型

from geometry_msgs.msg import Twist  # 用于发送速度命令到/cmd_vel话题
import time  # 导入time库，以使用sleep()函数
```

这里，重点是`ActionServer`对象的导入，它使得创建action server节点成为可能。使用`Twist`接口从`geometry_msgs.msg`来发送速度命令，并通过`time`库来实现时间间隔的控制。

**定义MyActionServer类**

```python
class MyActionServer(Node):
    def __init__(self):
        super().__init__('my_action_server')  # 初始化ROS 2节点，命名为my_action_server
        # 创建一个ActionServer对象
        self._action_server = ActionServer(self, Move, 'turtlebot3_as_2', self.execute_callback) 
        self.cmd = Twist()  # 初始化Twist消息
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)  # 定义一个Publisher发布到/cmd_vel话题
```

在构造函数中，初始化了一个名为`my_action_server`的ROS 2节点，并创建了一个`ActionServer`对象，这个对象是action通信的核心，允许服务器接收目标、发送反馈和结果。还定义了一个发布者，用于向`cmd_vel`话题发布`Twist`消息来控制机器人。

**execute_callback() 方法**

```python
def execute_callback(self, goal_handle):
    feedback_msg = Move.Feedback()  # 创建反馈消息实例
    feedback_msg.feedback = "Moving to the left left left..."  # 设置反馈消息内容

    # 根据目标请求的时长执行动作
    for i in range(1, goal_handle.request.secs):
        self.get_logger().info('Feedback: '.format(feedback_msg.feedback))  # 发布反馈消息
        goal_handle.publish_feedback(feedback_msg)  # 将反馈发布给客户端
        self.cmd.linear.x = 0.3  # 设置线速度
        self.cmd.angular.z = 0.3  # 设置角速度
        self.publisher_.publish(self.cmd)  # 发布速度命令
        time.sleep(1)  # 每次迭代后暂停一秒
```

这段代码演示了action的主要逻辑：根据目标请求的持续时间来执行动作（在这个例子中，使机器人移动）。通过在循环中发送速度命令，并在每次迭代后暂停，来控制机器人的移动。

- Publish the feedback message: **`goal_handle.publish_feedback(feedback_msg)`**
- Set the desired velocity values and publish this data: **`self.publisher_.publish(self.cmd)`**
- Finally, `sleep` for one second: **`time.sleep(1)`**

**停止机器人并返回结果**

```python
	goal_handle.succeed()  # 标记目标处理完成

    self.cmd.linear.x = 0.0  # 停止机器人
    self.cmd.angular.z = 0.0
    self.publisher_.publish(self.cmd)  # 发布停止命令

    result = Move.Result()  # 创建结果消息
    result.status = "Finished action server. Robot moved during 5 seconds"  # 设置结果消息的状态
    return result  # 返回结果给客户端
```

当动作完成时，将目标状态设置为成功，并将机器人的速度设置为0来停止机器人。然后创建一个结果消息，填充状态信息，并将其返回给客户端。

### 7.4  Create an Action Interface

To create your own Action Interface, you must complete the following three steps:

**1-** Create an **`action`** directory in your **`custom_interfaces`** package.

**2-** Create your **`<interface_name>.action`** Action Interface file. In this case `Move.action`.

- The name of the Action Interface file will later determine the name of the classes to be used in the **Action Server** and/or **Action Client**. ROS2 convention indicates that the name has to be camel-case.
- Remember that the Action Interface file has to contain three parts, each separated by three hyphens.

```
#goal
message_type goal_var_name
---
#result
message_type result_var_name
---
#feedback
message_type feedback_var_name
```

In this case, it should be like the following:

Move.action：

```
int32 secs
---
string status
---
string feedback
```

If you do not need one part of the message (for example, you do not need to provide feedback), then you can leave that part empty. However, you **must always specify the hyphen separators**.

**3-** Modify the **`CMakeLists.txt`** and **`package.xml`** files to include action interface compilation. Read the detailed description below.

#### 7.4.1  准备 CMakeLists.txt and package.xml

You have to edit two files in the package, in the same way as for Topics and Services:

- CMakeLists.txt
- package.xml

##### Modification of CMakeLists.txt

Call the **`rosidl_generate_interfaces`** function in your `CMakeLists.txt` file to create a new Action Interface. To call this function, add the below snippet to your CMakeLists.txt file:

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Move.action"
)
```

Within the function, specify the name of your Action Interface file.

If you have another interface in this package, you should include all the interfaces. **Do not delete** any interface.

In your case, if you have completed the previous units, you will probably have something like this:

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Age.msg"
  "srv/MyCustomServiceMessage.srv"
  "action/Move.action"
)
```

To generate Action Interfaces, ensure that you have access to the following packages:

- **`rosidl_default_generators`**
- **`action_msgs`**

For this, add the line below to your CMakeLists.txt file:

```cmake
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
```

As a reference, the **`CMakeLists.txt`** file of the **`t3_action_msg`** package, which contains the **`Move`** Action Interface, looks like this:

Note: `CMakeLists.txt` of `t3_action_msg package`.

```cmake
cmake_minimum_required(VERSION 3.5)
project(t3_action_msg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Move.action"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter, which checks for copyrights
  # uncomment the line when copyright and license are not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

##### Modification of package.xml

In the package.xml file, ensure that you have dependencies for the following packages:

- **`action_msgs`**
- **`rosidl_default_generators`**

```
<depend>action_msgs</depend>
<depend>rosidl_default_generators</depend>
```

##### compile

```
cd ~/ros2_ws
colcon build --packages-select custom_interfaces
source install/setup.bash
```

#### 7.4.2  Use the custom action interface in an action server and action client

This will be easy to test. You will modify your Action Server and Action Client programs created before. This time, instead of importing the action message from **`t3_action_msg`**, import it from your **`custom_interfaces`** package.

You could do it like this:

```
#from t3_action_msg.action import Move
from custom_interfaces.action import Move
```

Once you change these parts of the codes, compile with **`colcon build`** and launch both codes to see if it is working like it was working before.

```
cd ~/ros2_ws
colcon build
source install/setup.bash
```



## Action 总结

Action是ROS 2中的一种高级通信机制，专门设计用于处理需要反馈和可抢占性的长时间运行任务。通过Action Server和Action Client的配合使用，节点可以异步地执行任务，并且在任务执行过程中，客户端可以接收到服务器的实时反馈，也可以在需要时取消正在执行的任务。



Actions与Services在ROS 2中扮演类似的角色，都是基于客户端-服务器模型，用于实现节点间的功能调用。然而，Actions与Services之间存在两个关键的区别：

1. **可抢占性（Preemptable）**：Actions的一个主要特点是它们可以在执行过程中被取消或抢占。这意味着，如果客户端发起了一个Action，并且在该Action执行的任何时刻，客户端都可以决定取消这个Action。这与Services不同，后者一旦被调用，就会执行到完成，无法中途取消。
2. **反馈（Feedback）**：与Services相比，Actions提供执行过程中的反馈功能。当一个Action被执行时，服务器可以定期向客户端发送进度更新或其他类型的反馈信息，让客户端了解Action执行的当前状态。

在ROS 2中实现Action通信机制的工作流程大致如下：

- **客户端（Action Client）**发送一个目标（goal）给服务器（Action Server），这标志着Action的开始。
- **服务器（Action Server）**在Action执行过程中，可以向客户端（Action Client）发送反馈信息。
- 一旦Action完成，服务器将执行结果（response）返回给客户端。

在技术实现上，Actions利用Services来处理**目标（goal）**和**结果（result）**，同时使用Topics来处理反馈（feedback）。这种设计使Actions既能够支持复杂的、需要反馈的长期任务，也能够在必要时进行取消或抢占。

```
# goal
int32 secs # the number of seconds the robot will move forward
---
# result
string status # a string indicating the final status when the Action ends
---
# feedback
string feedback # a string that indicates the current status of the robot
```

### 1 Call Action server

不同类型的ROS 2通信机制使用的消息结构不同：

- **Topic的消息**只包含单一部分，即Topic提供的信息。

- **Service的消息**包含两部分：请求（request）和响应（response）。

- **Action Server的消息**则更为复杂，分为三个部分：目标（goal）、结果（result）和反馈（feedback）。

  目标（goal）：发送的控制信息，相当于flag，一般为控制时间。具体控制策略在Server中

  结果（result）：结果状态，执行完由Server发布, 只有一次

  反馈（feedback）：当前状态，每次周期都会接受

通过Action Client，和Action server进行通信。或者使用命令行**`ros2 action send_goal`**. 

```python
 ################# 创建ActionClient对象，用于与名为'turtlebot3_as'的action server通信#####
        # 参数解释：
        # self: 表示这个ActionClient关联的节点
        # Move: Action类型
        # 'turtlebot3_as': Action的名称，这需要与Action Server端匹配
        self._action_client = ActionClient(self, Move, 'turtlebot3_as')

        ##############################################################################
    def send_goal(self, seconds):
        # 创建一个Move.Goal消息对象
        goal_msg = Move.Goal()
        goal_msg.secs = seconds  # 设置目标消息的secs字段，表示需要执行动作的秒数

        # 等待action server变得可用
        self._action_client.wait_for_server()

        # 异步发送目标到action server，同时注册一个反馈回调函数
        # 参数解释：
        # goal_msg: 包含目标信息的消息对象
        # feedback_callback: 当接收到反馈时调用的回调函数
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        
        # 为send_goal_async返回的future对象添加一个完成回调
        # 当目标请求被处理（接受或拒绝）时调用
        self._send_goal_future.add_done_callback(self.goal_response_callback)
```

```
ros2 launch my_action_client example62_launch_file.launch.py
```

### 2 Writing an Action Server

接收action信息，并将其他信息publish给其他节点。

```python
################################### 创建ActionServer对象 #############################
        # 参数解释：
        # self: 表示这个ActionServer关联的节点
        # Move: Action类型
        # 'turtlebot3_as_2': Action的名称，客户端将通过此名称发送请求
        # self.execute_callback: 当接收到目标请求时执行的回调函数
        self._action_server = ActionServer(self, Move, 'turtlebot3_as_2', self.execute_callback)
        
        # 初始化用于发送移动命令的Twist消息
        self.cmd = Twist()
        # 创建一个发布者，用于发布Twist消息控制机器人
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
    
    # 执行目标的回调函数
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        ########################### 初始化feedback消息 ###############################
        feedback_msg = Move.Feedback()  #string
        feedback_msg.feedback = "Moving to the left left left..."
        
        # 根据目标请求的时长执行动作
        for i in range(1, goal_handle.request.secs):
            # 发送反馈信息
            self.get_logger().info('Feedback: {0} '.format(feedback_msg.feedback))
            goal_handle.publish_feedback(feedback_msg)
            
            # 设置机器人的移动命令
            self.cmd.linear.x = 0.3  # 前进速度
            self.cmd.angular.z = 0.3  # 转向速度
            
            # 发布移动命令
            self.publisher_.publish(self.cmd)
            time.sleep(1)  # 暂停一秒钟

        # 执行完成后停止机器人
        goal_handle.succeed()
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)
        
        ########################## 设置并返回执行结果 Result ###################
        result = Move.Result() #string
        result.status = "Finished action server. Robot moved during 5 seconds"
        self.get_logger().info('Result: {0}'.format(result.status))
        return result
```

- **Action的名称**是指在ROS系统中，客户端（Action Client）和服务器（Action Server）通过一个特定的名字进行通信的标识符。这个名称用于标识特定的Action服务，允许客户端知道它应该将目标（goal）发送到哪个服务器。在你的例子中，`'turtlebot3_as_2'`是Action的名称，用于客户端和服务器之间的匹配和通信。这个名字是自定义的。

- **ActionServer的名称**通常指的是创建ActionServer时给它的ROS节点的名称。在ROS中，每个节点都应该有一个唯一的名称，这样系统可以区分不同的节点。在提供的代码中，通过`super().__init__('my_action_server')`初始化的节点名称为`'my_action_server'`。这个名称是内部用来识别ROS节点的，而不是用于Action通信的标识符。

  不同的Action Server不能使用相同的Action名称进行通信。在ROS中，每个Action名称都对应一个唯一的Action Server，用于处理特定类型的Action请求。这种设计确保了客户端发送的目标（goal）能够准确地被送达到预期的服务器，并由该服务器处理。

### 3  Create an Action Interface

**1-** 创建 **`custom_interfaces`** package，然后创建 **`action`** 文件夹

**2-** 创建一个**`<interface_name>.action`** Action 接口文件.

 In this case `Move.action`.

我们这个actio叫**Move** （很重要，之后要用）

在.action文件中定义action信息类型，分为goal、result、feedback

```
#goal
message_type goal_var_name
---
#result
message_type result_var_name
---
#feedback
message_type feedback_var_name
```

3- CMakeLists.txt 

```
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Age.msg"
  "srv/MyCustomServiceMessage.srv"
  "action/Move.action"
)
```

```
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
```

4- 修改package.xml

添加以下两个依赖

- **`action_msgs`**
- **`rosidl_default_generators`**

```
<depend>action_msgs</depend>
<depend>rosidl_default_generators</depend>
```

```
<member_of_group>rosidl_interface_packages</member_of_group>
```

5- 使用自定义的interface

import：

```python
#from t3_action_msg.action import Move
from custom_interfaces.action import Move
```



## 8  ROS2 Debugging Tools

**What will you learn in this unit?**

- Add debugging ROS logs
- Basic use of RVIZ2 debugging tool
- view_frames tool
- ROS2 Doctor

**Knowing how to transform your thoughts and experience into real tasks is one of the most complex but crucial aspects of robotics**. In robotics programs, there is one constant: nothing works as it should. Since reality is far more complex, you'll need tools to figure out what's going on and where the problem lies. Therefore, debugging and visualization tools are critical in robotics, especially when dealing with complex data formats like **images, laser scans, pointclouds, or kinematic data**.

### 8.1  ROS2 Debugging Messages

Logs can be displayed on the screen, but they can also be saved in the ROS frame for sorting, filtering, and other uses. There are logging levels in systems, as seen in the image below. There are **five levels** in the case of ROS2 logs. Each level has a series of sub-levels. For example, if you use the **Error level**, the log will display the Error and **Fatal messages**. If your level is **Warning**, you'll see all of the messages for the levels **Warning, Error, and Fatal**.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_ROS2/images/debugging_msg.jpg)



- Create a new package named `logs_test`. When creating the package, add rclpy as dependency.
- Inside the `logs_test` folder of the package, create a new file named **`logger_example.py`**. Inside this file, copy the code shown below.
- Create a launch file named **`logger_example.launch.py`** for starting the new created node.
- Do the necessary modifications to your `setup.py` file, and compile the package.
- Execute the launch file.



### 8.3  tf2_tools view_frames

Another very useful tool is the **`view_frames`** program. This program allows you to visualize a tree diagram showing you the relationship between the different reference frames of your robot.

\- Example 8.2 -

Since the program will produce a PDF file in the directory where it is run, we suggest that you first move ouside of the ROS workspace:

```
cd
```

This command will return you to your home directory.

Now, execute the following instruction to have access to the ROS 2 commands:

```
source /opt/ros/humble/setup.bash
```



```
ros2 run tf2_tools view_frames
```

```
[INFO] [1671699921.424467830] [view_frames]: Listening to tf data for 5.0 seconds...
[INFO] [1671699926.430276901] [view_frames]: Generating graph in frames.pdf file...
[INFO] [1671699926.434963042] [view_frames]: Result:tf2_msgs.srv.FrameGraph_Response(frame_yaml="mp_400_fixed_wheel_right_link: \n  parent: 'base_footprint'\n  broadcaster: 'default_authority'\n  rate: 161.876\n  most_recent_transform: 2484.777000\n  oldest_transform: 2479.767000\n  buffer_length: 5.010\nbase_footprint: \n  parent: 'odom'\n  broadcaster: 'default_authority'\n  rate: 100.200\n most_recent_transform: 2484.767000\n  oldest_transform: 2479.767000\n  buffer_length: 5.000\nbase_link: \n  parent: 'base_footprint'\n  broadcaster: 'default_authority'\n  rate: 10000.000\n  most_recent_transform: 0.000000\n  oldest_transform: 0.000000\n  buffer_length: 0.000\nlidar_1_link: \nparent: 'base_link'\n  broadcaster: 'default_authority'\n  rate: 10000.000\n  most_recent_transform: 0.000000\n  oldest_transform: 0.000000\n  buffer_length: 0.000\nmp_400_caster_wheel_back_left_link: \n  parent: 'base_link'\n  broadcaster: 'default_authority'\n  rate: 10000.000\n  most_recent_transform: 0.000000\n  oldest_transform: 0.000000\n  buffer_length: 0.000\nmp_400_caster_wheel_back_right_link: \n  parent: 'base_link'\n  broadcaster: 'default_authority'\n  rate: 10000.000\n  most_recent_transform: 0.000000\n  oldest_transform: 0.000000\n  buffer_length: 0.000\nmp_400_caster_wheel_front_left_link: \n  parent: 'base_link'\n  broadcaster: 'default_authority'\n  rate: 10000.000\n  most_recent_transform: 0.000000\n  oldest_transform: 0.000000\n  buffer_length: 0.000\nmp_400_caster_wheel_front_right_link: \n  parent: 'base_link'\n  broadcaster: 'default_authority'\n  rate: 10000.000\n  most_recent_transform: 0.000000\n  oldest_transform: 0.000000\n  buffer_length: 0.000\nmp_400_fixed_wheel_left_link: \n  parent: 'base_footprint'\n  broadcaster: 'default_authority'\n  rate: 161.876\n  most_recent_transform: 2484.777000\n  oldest_transform: 2479.767000\n  buffer_length: 5.010\n")
```

Type the following to list all files in the current directory:

```
ls
```

You should get:

```
frames_2022-12-22_09.05.26.gv  frames_2022-12-22_09.05.26.pdf  ros2_ws
```

Using the IDE, you will also see that two new files have been created like the ones you see below:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_ROS2/images/frames.png)

Now, download the pdf file. You can do so by performing a right-click.

When you open it, you can visualize the tree diagram mentioned earlier.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_ROS2/images/frames-tree.png)

### 8.4  ROS2 Doctor

What happens when your ROS2 setup is not working as expected? Here you use ROS2's great tool, `ros2 doctor`.

This powerful tool reviews the total ROS2 setup from the platform, versions, network, environment, etc., in addition to giving you a proper diagnosis with precautions, errors, and possible reasons for problems.

Don't forget that `ros2doctor` belongs to the `ros2cli` package, so you will need to have it installed to use it.

```
ros2 doctor
```

```
ros2 doctor --report
```



## 9 Ros2 参数

类似C++编程中的全局变量，可以便于在多个程序中共享某些数据，**参数是ROS机器人系统中的全局字典，可以运行多个节点中共享数据。**

### **全局字典**

在ROS系统中，参数是以**全局字典**的形态存在的，什么叫字典？就像真实的字典一样，由名称和数值组成，也叫做键和值，合成**键值**。或者我们也可以理解为，就像编程中的参数一样，有一个参数名 ，然后跟一个等号，后边就是参数值了，在使用的时候，访问这个参数名即可。

**所以我们可以通过新建一个节点，用于集中修改其他节点中的参数值。**

### **可动态监控**

在ROS2中，参数的特性非常丰富，比如某一个节点共享了一个参数，其他节点都可以访问，如果某一个节点对参数进行了修改，其他节点也有办法立刻知道，从而获取最新的数值。这在**参数的高级编程**中，大家都可能会用到。

### **查看参数列表**

当前系统中有哪些参数呢？我们可以启动一个终端，并使用如下命令查询：

```
ros2 param list
```

### **参数查询与修改**

如果想要查询或者修改某个参数的值，可以在param命令后边跟get或者set子命令：

```
ros2 param describe turtlesim background_b   # 查看某个参数的描述信息
ros2 param get turtlesim background_b        # 查询某个参数的值
ros2 param set turtlesim background_b 10     # 修改某个参数的值
```

### **参数文件保存与加载**

一个一个查询/修改参数太麻烦了，不如试一试参数文件，ROS中的参数文件使用yaml格式，可以在param命令后边跟dump子命令，将某个节点的参数都保存到文件中，或者通过load命令一次性加载某个参数文件中的所有内容：

```
ros2 param dump turtlesim >> turtlesim.yaml  # 将某个节点的参数保存到参数文件中
ros2 param load turtlesim turtlesim.yaml     # 一次性加载某一个文件中的所有参数
```

### **参数编程**

#### **运行效果**

启动一个终端，先运行第一句指令，启动param_declare节点，终端中可以看到循环打印的日志信息，其中的“mbot”就是我们设置的一个参数值，参数名称是“robot_name”，通过命令行修改这个参数，看下终端中会发生什么？

```
ros2 run learning_parameter param_declare
ros2 param set param_declare robot_name turtle # 修改robot_name的值
```

#### **代码解析**

我们来看下在代码中，如何声明、创建、修改一个参数的值。

learning_parameter/param_declare.py

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2参数示例-创建、读取、修改参数
"""

import rclpy                                     # ROS2 Python接口库
from rclpy.node   import Node                    # ROS2 节点类

class ParameterNode(Node):
    def __init__(self, name):
        super().__init__(name)                                    # ROS2节点父类初始化
        self.timer = self.create_timer(2, self.timer_callback)    # 创建一个定时器（单位为秒的周期，定时执行的回调函数）
        self.declare_parameter('robot_name', 'mbot')              # 创建一个参数，并设置参数的默认值

    def timer_callback(self):                                      # 创建定时器周期执行的回调函数
        robot_name_param = self.get_parameter('robot_name').get_parameter_value().string_value   # 从ROS2系统中读取参数的值

        self.get_logger().info('Hello %s!' % robot_name_param)     # 输出日志信息，打印读取到的参数值

        new_name_param = rclpy.parameter.Parameter('robot_name',   # 重新将参数值设置为指定值
                            rclpy.Parameter.Type.STRING, 'mbot')
        all_new_parameters = [new_name_param]
        self.set_parameters(all_new_parameters)                    # 将重新创建的参数列表发送给ROS2系统

def main(args=None):                                 # ROS2节点主入口main函数
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = ParameterNode("param_declare")            # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口

```

完成代码的编写后需要设置功能包的编译选项，让系统知道Python程序的入口，打开功能包的setup.py文件，加入如下入口点的配置：

```
entry_points={
    'console_scripts': [
     'param_declare          = learning_parameter.param_declare:main',
    ],
},
```



























