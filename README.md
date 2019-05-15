# mecanum-kinova
#### 提供了全向底盘、kinova机械臂控制模块，以及联合控制模块

## mecanum
* 提供了mecanum串口报文协议，可通过有线或者无线串口控制全向底盘平移、旋转等

## kinova
* 提供了控制kinova的python程序(MICO_BASW.py)
* 提供了车载kinova的python程序（HighlevelRM.py），主要为抓取瓶子功能。提供了机械臂坐标系和车载kinect坐标系之间的转换。
* **需要对kinova和kinect进行坐标对齐**：见HighlevelRM.py 24-25行。
* 坐标对齐（方法找到空间中某点在kinova坐标系和kinect坐标系的坐标，作为对齐基准）：
  * 运行MICO_BASE.py中的demo2，计算机将实时输出当前机械臂前段的坐标信息，该坐标中心对应的是机械手两爪第一二关节连接处连线的中心。使用手柄，将机械臂移动到空间某一位置，可将一个瓶子的瓶盖置于机械臂坐标中心位置，记录下此刻的坐标。
  * 使用pykinect包中的定位功能，定位瓶盖中心坐标点
  * 至此，得到同一真实目标的两个坐标系坐标，填入HighlevelRM.py 24-25行
* 需要安装kinova驱动，安装方法查看相关文档
* 安装kinova驱动注意事项：首先运行kinova驱动安装程序，然后插上kinova设备，在设备管理器中找到该设备并‘更新驱动’，路径指定到刚刚驱动安装的目录。
* **特别注意**，这里应当先**禁用windows系统驱动数字签名**,具体方法查看百度（开机F8或者其他）


* 将CommandLayerWindows.dll和CommunicationLayerWindows.dll复制到python根目录下，我们的程序即可运行
* **特别注意**：以上两个dll均为32位，因此**kinova程序仅支持32位python

## mecanum-kinova
* 提供了联合操控全向底盘和机械臂的程序
* 由于kinova仅支持32位python，因此程序提供了两个类，rmcar_x86和rmcar_x64，x86版本可直接使用32位python运行，x64版本提供64位python运行，在x64中，我们将使用32位python启动rmcar-server.py，因此，需要安装32位python并添加环境变量，且程序中可能需要适当修改rmcar_x64初始化中的subprocess.Popen('python2 E:\MobileRobot\RmCar\RmCar_server.py')代码，确保能够启动32位python，且server路径正确。

## 包的安装
* 下载文件后，直接将mecanum-kinova.pth文件中的路径修改为对应文件夹的绝对路径，并将.pth文件放置在python_path\Lib\site-packages下
