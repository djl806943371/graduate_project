# ROS
上位机程序，用于控制轮毂电机及下位机

一、安装、配置
    1.添加源
        $ sudo sh -c 'echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    2.设置秘钥
        $ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    3.下载安装
        $ sudo apt update
        $ sudo apt install ros-melodic-desktop-full
    4.初始化
        $ sudo rosdep init
        $ rosdep update
    5.管理环境变量
        .bashrc中添加ros初始化命令
            $ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
        /opt/ros/melodic/setup.bash中添加环境变量：
            export ROS_HOSTNAME=localhost
        如果要在当前terminal操作执行如下语句（其实重新开启终端即可）：
            $ source ~/.bashrc

二、ROS文件系统工具
    # rospack find [包名称]
        $ rospack find roscpp
    # roscd [本地包名称[/子目录]]
        $ roscd roscpp
    # roscd log
        $ roscd log可以切换到ROS保存日记文件的目录下。需要注意的是，如果你没有执行过任何ROS程序，系统会报错说该目录不存在。
    # rosls [本地包名称[/子目录]]
        $ rosls roscpp_tutorials

三、创建 ROS catkin 工作空间
    1.创建文件夹并进入
        $ mkdir -p ~/djl/graduate_project/ros/catkin_ws/src
        $ cd ~/djl/graduate_project/ros/catkin_ws
    2.即使这个工作空间是空的（在'src'目录中没有任何软件包，只有一个CMakeLists.txt链接文件），你依然可以编译它：
        $ catkin_make
    3.把当前工作空间添加到/opt/ros/melodic/setup.bash中
        ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/jiaolongdu/djl/graduate_project/ros/catkin_ws/src
    4.一个简单的工作空间也许看起来像这样：
        workspace_folder/               -- WORKSPACE
            src/                        -- SOURCE SPACE
                CMakeLists.txt          -- 'Toplevel' CMake file, provided by catkin
                package_1/
                    CMakeLists.txt      -- CMakeLists.txt file for package_1
                    package.xml         -- Package manifest for package_1
                ...
                package_n/
                    CMakeLists.txt      -- CMakeLists.txt file for package_n
                    package.xml         -- Package manifest for package_n

四、创建catkin程序包
    1.进入catkin工作空间的src/文件夹
        $ cd ~/djl/graduate_project/ros/catkin_ws/src
    2.创建名为beginner_tutorials的程序包，该程序包依赖std_msgs、roscpp和rospy
        格式：catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
            $ catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
    3.查看直接依赖：
        $ rospack depends1 beginner_tutorials
    4.查看包含直接依赖和间接依赖的所有依赖：
        $ rospack depends beginner_tutorials
五、编译程序包
    1.进入catkin工作空间
        $ cd ~/djl/graduate_project/ros/catkin_ws/
    2.下述命令会编译src文件夹下的所有catkin工程
        $ catkin_make
        $ catkin_make install  #(可选)
    3.catkin工作空间会产生 build/ 目录和 devel/ 目录。build/ 目录是build space的默认所在位置，同时cmake 和 make也是在这里被调用来配置并编译你的程序包。devel/ 目录是devel space的默认所在位置, 同时也是在你安装程序包之前存放可执行文件和库文件的地方。
六、ROS节点
    1.ROS中图的概念：
        Nodes:节点,一个节点即为一个可执行文件，它可以通过ROS与其它节点进行通信。
        Messages:消息，消息是一种ROS数据类型，用于订阅或发布到一个话题。
        Topics:话题,节点可以发布消息到话题，也可以订阅话题以接收消息。
        Master:节点管理器，ROS名称服务 (比如帮助节点找到彼此)。
        rosout: ROS中相当于stdout/stderr。
        roscore: 主机+ rosout + 参数服务器 (参数服务器会在后面介绍).
    2.客户端库
        ROS客户端库允许使用不同编程语言编写的节点之间互相通信:
            rospy = python 客户端库
            roscpp = c++ 客户端库
    3.运行主机，roscore 是你在运行所有ROS程序前首先要运行的命令。
        $ roscore
    4.运行某个程序包内的一个节点
        格式：rosrun [package_name] [node_name]
            $ rosrun turtlesim turtlesim_node
    5.查看当前运行的ROS节点信息
        $ rosnode list
    6.查看关于一个特定节点的信息
        $ rosnode info /rosout
    7.可以通过命令行重新配置节点名称
        $ rosrun turtlesim turtlesim_node __name:=my_turtle
    8.测试节点ping值
        $ rosnode ping my_turtle
七、ROS话题
    1.运行一个节点，作为被控节点
        $ rosrun turtlesim turtlesim_node
    2.运行另一个节点，作为控制节点，通过键盘远程控制turtle
        $ rosrun turtlesim turtle_teleop_key
    3.通过命令可以查看当前话题发布及订阅关系
        $ rosrun rqt_graph rqt_graph
    4.rostopic echo可以显示在某个话题上发布的数据。
        rostopic echo [topic]
            rostopic echo /turtle1/cmd_vel
    5.rostopic list能够列出所有当前订阅和发布的话题。
        $ rostopic list
    6.rostopic type 命令用来查看所发布话题的消息类型
        $ rostopic type /turtle1/cmd_vel
    7.rosmsg 命令来查看消息的详细情况
        $ rosmsg show geometry_msgs/Twist
    8.rostopic pub可以把数据发布到当前某个正在广播的话题上
        格式：rostopic pub [topic] [msg_type] [args]
            发布单条消息后退出：
                $ rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
            发布稳定命令流：
                $ rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
    9.rostopic hz命令可以用来查看数据发布的频率
        $ rostopic hz /turtle1/pose
    10.命令可以组合
        $ rostopic type /turtle1/cmd_vel | rosmsg show
    11.rqt_plot命令可以实时显示一个发布到某个话题上的数据变化图形
        $ rosrun rqt_plot rqt_plot
八、ROS服务
    服务（services）是节点之间通讯的另一种方式。服务允许节点发送请求（request） 并获得一个响应（response）。
        rosservice可以很轻松的使用 ROS 客户端/服务器框架提供的服务。
            1.rosservice list         输出可用服务的信息
                rosservice type [service]
                    $ rosservice type clear
            2.rosservice call         调用带参数的服务
                rosservice call [service] [args]
                    $ rosservice call clear             #服务清除了turtlesim_node的背景上的轨迹
            3.rosservice type         输出服务类型
                rosservice type [service]
                    $ rosservice type clear
            4.rosservice find         依据类型寻找服务find services by service type
                rosservice uri          输出服务的ROSRPC uri
        rosparam使得我们能够存储并操作ROS 参数服务器（Parameter Server）上的数据.
            1.rosparam list           列出参数名
                $ rosparam list
            2.rosparam set            设置参数
                rosparam set [param_name]
                    $ rosparam set background_r 150
            3.rosparam get            获取参数
                rosparam get [param_name]
                    $ rosparam get background_r
                    $ rosparam get /        #显示全部参数
            4.rosparam load           从文件读取参数
                rosparam load [file_name] [namespace]
                    $ rosparam load params.yaml copy    #将yaml文件重载入新的命名空间
                    $ rosparam get copy/background_b
            5.rosparam dump           向文件中写入参数
                rosparam dump [file_name]
                    $ rosparam dump params.yaml
            6.rosparam delete         删除参数
九、rqt_console 和 roslaunch
    1.rqt_console属于ROS日志框架(logging framework)的一部分，用来显示节点的输出信息。rqt_logger_level允许我们修改节点运行时输出信息的日志等级（logger levels）（包括 DEBUG、WARN、INFO和ERROR）。
        $ rosrun rqt_console rqt_console
        $ rosrun rqt_logger_level rqt_logger_level        
    2.roslaunch可以用来启动定义在launch文件中的多个节点。
        roslaunch [package] [filename.launch]
            $ mkdir launch
            $ cd launch
            现在我们来创建一个名为turtlemimic.launch的launch文件并复制粘贴以下内容到该文件里面：
                1 <launch>
                2 
                3   <group ns="turtlesim1">
                4     <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
                5   </group>
                6 
                7   <group ns="turtlesim2">
                8     <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
                9   </group>
                10 
                11   <node pkg="turtlesim" name="mimic" type="mimic">
                12     <remap from="input" to="turtlesim1/turtle1"/>
                13     <remap from="output" to="turtlesim2/turtle1"/>
                14   </node>
                15 
                16 </launch>
            $ roslaunch beginner_tutorials turtlemimic.launch
十、消息(msg)和服务(srv)介绍
    消息(msg): msg文件就是一个描述ROS中所使用消息类型的简单文本。它们会被用来生成不同语言的源代码。
    服务(srv): 一个srv文件描述一项服务。它包含两个部分：请求和响应。
    msg文件存放在package的msg目录下，srv文件则存放在srv目录下。
    msg文件实际上就是每行声明一个数据类型和变量名。可以使用的数据类型如下：
        int8, int16, int32, int64 (plus uint*)
        float32, float64
        string
        time, duration
        other msg files
        variable-length array[] and fixed-length array[C]
        在ROS中有一个特殊的数据类型：Header，它含有时间戳和坐标系信息。在msg文件的第一行经常可以看到Header header的声明.
    下面是一个msg文件的样例，它使用了Header，string，和其他另外两个消息类型。
        Header header
        string child_frame_id
        geometry_msgs/PoseWithCovariance pose
        geometry_msgs/TwistWithCovariance twist
    srv文件分为请求和响应两部分，由'---'分隔。下面是srv的一个样例：
        int64 A
        int64 B
        ---
        int64 Sum
    1.创建消息
        接下来，还有关键的一步：我们要确保msg文件被转换成为C++，Python和其他语言的源代码：
        查看package.xml, 确保它包含一下两条语句:

        <build_depend>message_generation</build_depend>
        <exec_depend>message_runtime</exec_depend>
        如果没有，添加进去。 注意，在构建的时候，我们只需要"message_generation"。然而，在运行的时候，我们只需要"message_runtime"。

        在你最喜爱的编辑器中打开CMakeLists.txt文件(可以参考前边的教程rosed).

        在 CMakeLists.txt文件中，利用find_packag函数，增加对message_generation的依赖，这样就可以生成消息了。 你可以直接在COMPONENTS的列表里增加message_generation，就像这样：

        # Do not just add this line to your CMakeLists.txt, modify the existing line
        find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)
        有时候你会发现，即使你没有调用find_package,你也可以编译通过。这是因为catkin把你所有的package都整合在一起，因此，如果其他的package调用了find_package，你的package的依赖就会是同样的配置。但是，在你单独编译时，忘记调用find_package会很容易出错。

        同样，你需要确保你设置了运行依赖：

        catkin_package(
            ...
            CATKIN_DEPENDS message_runtime ...
        ...)
        找到如下代码块:

        # add_message_files(
        #   FILES
        #   Message1.msg
        #   Message2.msg
        # )
        去掉注释符号#，用你的.msg文件替代Message*.msg，就像下边这样：
            add_message_files(
                FILES
                Num.msg
            )
        手动添加.msg文件后，我们要确保CMake知道在什么时候重新配置我们的project。 确保添加了如下代码:
        generate_messages()