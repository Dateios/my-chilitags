### 软件安装

- 摄像头调试软件

    在ubuntu软件中搜索guvcview下载安装

- 下载vscode

    `sudo add-apt-repository ppa:ubuntu-desktop/ubuntu-make`

    `sudo apt-get update`

    `sudo apt-get install ubuntu-make`

- 下载opencv3.2

### 运行指令

- 试运行程序

    使用`su -`进入终端，不要使用`su root`,否则会报错

    `cd my-chilitags`

    `mkdir build && cd build`

    `ccmake..`(注意激活with_samples选项)

    `make`

    `sudo make install`

    `./samples/challenge 1280 720 1 2`(如果看到“hahah”，表示程序运行成功)

- 修改程序后运行

    `make && ./samples/challenge 1280 720 1 2`

- 参数解析

    第一个第二个参数（1280 720） 摄像头参数

    第三个参数（一般是1） 代表摄像头编号

    第四个参数（0,1,2）0代表挑战赛一，1代表挑战赛二，2代表调式

- 程序闪退

    使用guvcview调整曝光（100左右）白平衡（3250左右），整体色调会偏蓝

    可以通过调整曝光度来让视觉识别更加准确

### 通信

- 连接机器人

    挑战赛一中，一号机器人使用MF，二号机器人使用GP

    挑战赛二中，只有二号机器人

    一号机器人通讯编号1,二号机器人通讯编号0，所以要先连接GP，再连接MF

    代码详见：

    `ourRobotOne.robot_num = 1;`

    `ourRobotTwo.robot_num = 0;`

- 连接方式

    使用蓝牙2.0

### 视觉

- 场地

    测量场地的长GOUND_H、宽GOUND_W（一般不会有错，340X180）

    测量摄像头高度CAM_H、机器人高度ROBOT_H（注意减去球的6cm）

    记录场地左上角坐标g_left、右下角坐标g_left

    在场地上按鼠标中键（会打印出经过坐标转换后的场地坐标）

    检查转换后坐标是否正确，主要检测左上角是否为(0,0)，右下角是否为(340,180)

    如果不正确，一般是Y值会偏移，修改YOFFSET为正数或者负数，再检查转换后的坐标是否正确

    代码详见：

    `#define GOUND_H 340 `

    `#define GOUND_W 180`

    `#define YOFFSET 0`

    `#define CAM_H 284//-6`

    `#define ROBOT_H 27//-6`

- 色标

    两个挑战赛中，我们都使用蓝色色标，障碍物机器人使用黄色色标

    色标能够识别的表现是能框出色标，中间显示识别的数字，并能显示front表示正前方

    挑战赛中，一号机器人使用14号色标，二号机器人使用234号色标，一号障碍物使用99号色标，二号障碍物使用14号色标

    代码详见：

    `#define ROBOTONE 14`

    `#define ROBOTTWO 234`

    `#define OBST_ONE 99`

    `#define OBST_TWO 14`

- 球

    使用圆形框出，还会画出球附近的区域


