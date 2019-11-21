安装 scan_tools 需要安装 csm
安装csm需要安装sudo apt install libgsl-dev
安装csm时不要在catkin工作空间内，安装步骤：
    git clone https://github.com/AndreaCensi/csm.git
    cd csm
    cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr/local . 
    make 
    sudo make install
安装完成后catkin_make

安装navigation包需要先:
    sudo apt install ros-melodic-navigation

解决包依赖问题:
    rosdep install --from-paths src --ignore-src -r -y