/*******************************************************************************
*   Copyright 2013-2014 EPFL                                                   *
*   Copyright 2013-2014 Quentin Bonnard                                        *
*                                                                              *
*   This file is part of chilitags.                                            *
*                                                                              *
*   Chilitags is free software: you can redistribute it and/or modify          *
*   it under the terms of the Lesser GNU General Public License as             *
*   published by the Free Software Foundation, either version 3 of the         *
*   License, or (at your option) any later version.                            *
*                                                                              *
*   Chilitags is distributed in the hope that it will be useful,               *
*   but WITHOUT ANY WARRANTY; without even the implied warranty of             *
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
*   GNU Lesser General Public License for more details.                        *
*                                                                              *
*   You should have received a copy of the GNU Lesser General Public License   *
*   along with Chilitags.  If not, see <http://www.gnu.org/licenses/>.         *
*******************************************************************************/

// This file serves as an illustration of how to use Chilitags


// The Chilitags header
#include <chilitags.hpp>

#ifdef OPENCV3
#include <opencv2/core/utility.hpp> // getTickCount...
#include <opencv2/imgproc/imgproc.hpp>
#endif

#include <opencv2/core/core_c.h> // CV_AA
#include <opencv2/core/core.hpp>

// OpenCV goodness for I/O
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <csignal>
#include <vector>

using namespace cv;

#define IMAGE_H 1280
#define IMAGE_W 720

#define GOUND_H 320
#define GOUND_W 180

// 需要实际测量的值
#define YOFFSET 5
#define CAM_H 284
#define ROBOT_H 32

#define ROBOTONE 14//14
#define ROBOTTWO 515//99
#define OBST_ONE 14
#define OBST_TWO 515


#define PI 3.141592653

bool sRunning = true;
cv::VideoCapture capture;

static cv::Point g_left(60,60);//场地左上角坐标  
static cv::Point g_right(1140,640);//场地右下角坐标

static cv::Point2f b_center(-1.0f,-1.0f);//球的坐标
static float b_radius = 0.0f;
static cv::Point b_position(-1, -1);

static cv::Point b_init(-1,-1);

static cv::Point ground[IMAGE_H][IMAGE_W] = {};

static int64 startTime = 0;
static int64 endTime = 0;

static int64 commandStartTime[3] = {};
static int64 commandEndTime[3] = {};

static int64 b_startrecord = 0;
static int64 b_endrecord = 0;

//机器人指令1,0-3为方向动作(前后左右),4-9为6种动作
const char * dir0[15][12] = {
{"ff","55","00","ff","00","ff"},  //停止 0
{"ff","55","01","fe","00","ff","ff","55","00","ff","00","ff"},  //Up	前进  1
{"ff","55","02","fd","00","ff","ff","55","00","ff","00","ff"},  //Down	后退  2
{"FF","55","05","FA","00","FF"},	//L+U   前移左转    3
{"FF","55","09","F6","00","FF"},	//R+U   前移右转    4
{"ff","55","04","fb","00","ff"},  	//Left	左转  5
{"ff","55","08","f7","00","ff"},  	//Right	右转  6

{"ff","55","04","FB","01","FE"},  	//L+5	左移  7
{"ff","55","04","FB","03","FC"},	//L+5+6	快速左移    8
{"ff","55","08","F7","01","FE"},	//R+5	右移  9
{"ff","55","08","F7","03","FC"},	//R+5+6	快速右移    10

{"ff","55","05","FA","01","FE"},	//L+U+5	左前移 11
{"FF","55","09","F6","01","FE"},	//R+U+5 右前移 12
{"ff","55","21","DE","00","FF"},	//2+U	左脚前踢    13
{"ff","55","81","71","00","ff"}	    //4+U	右脚前踢    14
};

//机器人指令2
const char dir1[23][3] = {"00","01","02","03","04",	//0-4	停止,前进,后退,左转,右转
 		"05","06","07","08","09",		//5-9	左移,右移,左踢,右踢，前摔站立
		"0a","0b","0c","0d","0e","0f",  //10-15	后摔站立,narrow walk,左大步走，右大步走，stand pose，one_step
        "13","14","15","16","18","19","20"};      //16-22 左侧踢，右侧踢,back narrow walk,左转20, 右转20, 左转45度, back_shoot			

struct RobotInfo{
    int number;
    cv::Point position;
    cv::Point g_position;
    cv::Point front;
    double theta;
};

RobotInfo ourRobotOne;
RobotInfo ourRobotTwo;

RobotInfo obstacle_One;
RobotInfo obstacle_Two;

//将 字符串型 数组转换为 char型 数组,其表示的值不变.如:
//输入:{"ff","dd"};输出:{'255','221'}
void str_to_ch(const char ** input,char * output,int length )
{
    int tem;
    for(int i = 0;i < length;i++)
    {
        tem = strtol(input[i],NULL,16);
        output[i] = tem;
    }
}

//将二进制编码数组写入设备文件
//如:{'255','221'} 写入 /dev/rfcomm0
void write_file(FILE * file,char *input,int length)
{
    int num = 0;
    for(int i = 0;i < length;i++)
    {
        if( num = fprintf(file,"%c",input[i]) < 1)
            printf("文件写入失败!\n");
    }
}

/*
功能:向指定机器人发送指定指令,如:(1,10,1),1号机器人做2号动作,指令类型2
参数:
robot:1,2  1-> /dev/rfcomm0  2-> /dev/rfcomm1
direction:0-9(12)
type:1,2    1-> 12字节指令  2-> 1字节指令
*/
void send(int robot_num , int direction , int type)
{
    FILE * file;
    char  send[12];
    char filename0[30] = "/dev/rfcomm0";
    char filename1[30] = "/dev/rfcomm1";
    char filename2[30] = "/dev/rfcomm2";
    float timeout = 0;

    if(type == 1)
    {
        timeout = 0.0f;
    }
    else
    {
        timeout = 1.0f;
    }
	
	//判断机器人编号
    if(robot_num == 0 )
    {
        if( ( file = fopen(filename0,"w+")) == NULL)
        {
            printf("%s \n","设备0读取失败!");
        }
        commandEndTime[0] = cv::getTickCount();
    }
    else if(robot_num == 1)
    {
        if( ( file = fopen(filename1,"w+")) == NULL)
        {
            printf("%s \n","设备1读取失败!");
        }
        commandEndTime[1] = cv::getTickCount();
    }
    else
    {
        if( ( file = fopen(filename2,"w+")) == NULL)
        {
            printf("%s \n","设备2读取失败!");
        }
        commandEndTime[2] = cv::getTickCount();
    }

    float exeTime[3] = {}; 
    for(int i = 0; i < sizeof(exeTime)/sizeof(float); i++)
    {
        // printf("%i \n", sizeof(exeTime)/sizeof(float));  
	    exeTime[i]  = ((float) commandEndTime[i] - commandStartTime[i])/cv::getTickFrequency();
        // printf("%i \n", exeTime[i] > 1.0f);
        if(exeTime[i] > timeout)
        {
            commandEndTime[i] = cv::getTickCount();
            //判断机器人指令类别
            if(type == 1)
            {
                if(dir0[direction][6] != NULL)
                {
                    str_to_ch(dir0[direction], send, 12);
                    write_file(file, send, 12);
                    commandStartTime[i] = commandEndTime[i];
                    printf("发送一个12 ! \n");
                }
                else if(dir0[direction][6] == NULL)
                {
                    str_to_ch(dir0[direction], send, 6);
                    write_file(file, send, 6);
                    commandStartTime[i] = commandEndTime[i];
                    printf("发送一个6 ! \n");
                }
                else
                {
                    printf("指令错误 ! \n");
                }
            }
            else
            {
                if(direction >= 0 && direction < 21)
                {
                    int tem = strtol(dir1[direction],NULL,16);
                    unsigned char ch = (unsigned char)tem;
                    fprintf(file,"%c",ch);
                    commandStartTime[i] = commandEndTime[i];
                    printf("发送一个1 ! \n");
                }
                else
                {
                    printf("指令错误 ! \n");
                }
            }
        }
    }
    
	//发送指令
    fclose(file);
}

void quitFunction(int sig)
{
    std::cout << "Caught interrupt. Closing" << std::endl;
    capture.release();
    sRunning = false;
}

void on_mouse(int event,int x,int y,int flags,void *ustc)//event鼠标事件代号，x,y鼠标坐标，flags拖拽和键盘操作的代号  
{   
    if (event == CV_EVENT_LBUTTONDOWN)//左键按下，读取初始坐标  
    {   
        g_left = cv::Point(x,y);  
    }
    else if (event == CV_EVENT_LBUTTONUP)//左键松开，将在图像上划矩形  
    {   
        g_right = cv::Point(x,y);
        //std::cout << g_left<<"  " <<g_right << std::endl;
        //进行坐标转换
        int xk = g_right.x - g_left.x;
        int yk = g_right.y - g_left.y;
        for(int i = 0; i < IMAGE_H; i++)
        {
            for(int j = 0; j < IMAGE_W; j++)
            {
                if(i > g_left.x || j > g_left.y || i < g_right.x || j < g_right.y)
                {
                    ground[i][j].x = GOUND_H * (i - g_left.x) / xk;
                    ground[i][j].y = GOUND_W * (j - 720 + g_right.y) / yk + YOFFSET;
                }
            }
        }
    }
    else if (event == CV_EVENT_MBUTTONDOWN)//中键获取当前点的坐标
    {
        std::cout << ground[x][y] << std::endl;
    }  
}

double getAngle(cv::Point2f pSrc, cv::Point2f p1, cv::Point2f p2)
{
    double angle = 0.0f;

    double va_x = p1.x - pSrc.x;
    double va_y = p1.y - pSrc.y;

    double vb_x = p2.x - pSrc.x;
    double vb_y = p2.y - pSrc.y;

    double va_vb = (va_x * vb_x) + (va_y * vb_y);
    double vaxvb = (va_x * vb_y) - (vb_x * va_y);

    double va_val = sqrt(va_x * va_x + va_y * va_y);
    double vb_val = sqrt(vb_x * vb_x + vb_y * vb_y);

    double cosVal = va_vb / (va_val * vb_val);
    double sinVal = vaxvb / (va_val * vb_val);

    if(cosVal < -1 && cosVal > -2)
        cosVal = -1;
    else if(cosVal > 1 && cosVal <2)
        cosVal = 1;
    
    angle = acos(cosVal) * 180 / PI;

    // 逆时针
    if(sinVal < 0)
        angle = 360 - angle;

    return angle;
}

static inline double getDistance(cv::Point2f p1, cv::Point2f p2)
{
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

void drawTags(
    cv::Mat outputImage,
    const chilitags::TagCornerMap &tags,
    bool isour
    ){
    cv::Scalar COLOR = cv::Scalar(255, 0, 255);
    cv::Scalar ROBOT_COLOR = cv::Scalar(255, 255, 0);

    for (const auto & tag : tags) {

        const cv::Mat_<cv::Point2f> corners(tag.second);
        cv::Point2f center = 0.5f*(corners(0) + corners(2));
        cv::Point2f front = 0.5f*(corners(0) + corners(1));
        cv::Point2f posit(center.x + 10.0f , center.y);
        cv::Point g_position(-1, -1);
        cv::Point g_front(-1, -1);

        // 位置补偿
        cv::Point2f g_center = ground[640][360];
        cv::Point2f g_center_r = ground[(int)center.x][(int)center.y];
        cv::Point2f g_front_r = ground[(int)front.x][(int)front.y];
        cv::Point2f g_posit = ground[(int)posit.x][(int)posit.y];
        float robot2center = getDistance(g_center, g_center_r);
        float g_offset = (ROBOT_H * robot2center) / (CAM_H - ROBOT_H);

        // 注意要把角度值转换成弧度值
        double r2c_angle =  getAngle(g_center_r, g_center, g_posit) * 2 * PI / 360;
        double sinr2c = g_offset * sin(r2c_angle);
        double cosr2c = g_offset * cos(r2c_angle);

        if(r2c_angle <= PI / 2)
        {
            g_position.x = g_center_r.x + cosr2c;
            g_position.y = g_center_r.y - sinr2c;

            g_front.x = g_front_r.x + cosr2c;
            g_front.y = g_front_r.y - sinr2c;
        }
        else if(r2c_angle >  PI / 2 && r2c_angle <= PI)
        {
            g_position.x = g_center_r.x - (-cosr2c);
            g_position.y = g_center_r.y - sinr2c;

            g_front.x = g_front_r.x - (-cosr2c);
            g_front.y = g_front_r.y - sinr2c;
        }
        else if(r2c_angle > PI && r2c_angle <= (3/2) * PI)
        {
            g_position.x = g_center_r.x - (-cosr2c);
            g_position.y = g_center_r.y + (-sinr2c);

            g_front.x = g_front_r.x - (-cosr2c);
            g_front.y = g_front_r.y + (-sinr2c);
        }
        else
        {
            g_position.x = g_center_r.x + cosr2c;
            g_position.y = g_center_r.y + (-sinr2c);

            g_front.x = g_front_r.x + cosr2c;
            g_front.y = g_front_r.y + (-sinr2c);
        }

        // 获取我方机器人的位置和角度14
        if(isour)
        {
            ROBOT_COLOR = cv::Scalar(255, 255, 0);
            if(tag.first == ROBOTONE)
            {
                // std::cout << sinr2c << " " << cosr2c << std::endl;
                // std::cout << r2c_angle << std::endl;
                // std::cout << g_center_r << std::endl;
                ourRobotOne.position = center;
                ourRobotOne.g_position = g_position;
                ourRobotOne.front = g_front;
                ourRobotOne.theta = getAngle(center, front, posit);
                // std::cout << g_position << std::endl;
                // std::cout << b_position << std::endl;
            }
            else if(tag.first == ROBOTTWO)
            {
                ourRobotTwo.position = center;
                ourRobotTwo.g_position = g_position;
                ourRobotTwo.front = g_front;
                ourRobotTwo.theta = getAngle(center, front, posit);
                // std::cout << g_center_r.x << std::endl;
                // std::cout << "---" << ground[(int)center.x][(int)center.y] << std::endl;
            }
        }
        else
        {
            ROBOT_COLOR = cv::Scalar(0, 255, 255);
            if(tag.first == OBST_ONE)
            {
                // obstacle_One.g_position = g_position; 
            }
            else if(tag.first == OBST_TWO)
            {
                // obstacle_Two.g_position = g_position;
            }
        }
        

        for (size_t i = 0; i < 4; ++i) {
            static const int SHIFT = 16;
            static const float PRECISION = 1<<SHIFT;
            cv::line(
                outputImage,
                PRECISION*corners(i),
                PRECISION*corners((i+1)%4),
#ifdef OPENCV3
                ROBOT_COLOR, 1, cv::LINE_AA, SHIFT);
#else
                ROBOT_COLOR, 1, CV_AA, SHIFT);
#endif
        }
        
        cv::putText(outputImage, cv::format("%s", "front"), front,
                    cv::FONT_HERSHEY_SIMPLEX, 0.2f, COLOR);
        cv::putText(outputImage, cv::format("%d", tag.first), center,
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, COLOR);
    }

    // float processingTime = 1000.0f*((float) endTime - startTime)/cv::getTickFrequency();

    cv::putText(outputImage,
                cv::format("%dx%d (press q to quit)",
                            outputImage.cols, outputImage.rows),
                cv::Point(32,32),
                cv::FONT_HERSHEY_SIMPLEX, 0.5f, COLOR);
}

static inline bool ContoursSortByArea(std::vector<cv::Point> contour1, std::vector<cv::Point> contour2)
{
    return (cv::contourArea(contour1) > cv::contourArea(contour2));
}

static inline bool isBallMove()
{
    static bool bRecord = false;

    // 只对球的初始位置记录一次
    if(b_init.x != -1)
        bRecord = true;
    if(!bRecord)
        b_init = b_position;
    if(getDistance(b_init, b_position) > 10 && getDistance(b_init, b_position) < 20)
        return true;
    else
        return false;
}

void drawBall(cv::Mat image)
{
    cv::Mat hsvImage, mask;
    std::vector<std::vector<cv::Point>> contours;
    
    // 转换为HSV图像
    cv::cvtColor(image, hsvImage, cv::COLOR_BGR2HSV);
    
    // 创建一个红色的mask
    // 红色cv::Scalar(0, 43, 46), cv::Scalar(10, 255, 255)
    cv::inRange(hsvImage, cv::Scalar(35, 43, 46), cv::Scalar(77, 255, 255), mask);
    
    // 腐蚀和膨胀
    cv::erode(mask, mask, 2);
    cv::dilate(mask, mask, 2);

    // cv::namedWindow("mask");
    // cv::imshow("mask", mask);

    // 找到mask中的轮廓
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 根据轮廓面积排序
    // Point2f b_center;
    // float b_radius;
    std::sort(contours.begin(), contours.end(), ContoursSortByArea);

    // std::cout << cv::contourArea(contours[0]) << std::endl;

    // 删除面积过大的轮廓
    std::vector<std::vector<cv::Point>>::iterator iter = contours.begin();
    for( ; iter != contours.end(); )
    {
        // 寻找最小包围圆形
        cv::minEnclosingCircle(*iter, b_center, b_radius);
        
        // 如果圆心在场地外
        if(b_center.x < g_left.x || b_center.y < g_left.y || b_center.x > g_right.x || b_center.y > g_right.y)
        {
            iter++;
            continue;
        }
        else
        {
            if(cv::contourArea(*iter) > 800)
                contours.erase(iter);
            // else if(cv::contourArea(*iter) < 200)
            //     break;
            else
            {
                // std::cout << cv::contourArea(*iter) << std::endl;
                // 绘制轮廓
                //cv::drawContours(image, contours, 0, Scalar(255, 0, 255), -1);
                // std::cout << cv::contourArea(*iter) << std::endl;
                b_position = ground[(int)b_center.x][(int)b_center.y];
                break;
            }
        }
    }    
}

void coverIt(const cv::Mat& image, const cv::Scalar lower, cv::Scalar upper)
{
    
    cv::Mat hsvImage, mask, outImage;
    std::vector<std::vector<cv::Point>> contours;
    cv::Point2f corners[4];
    cv::Point2f center;
    float radius;
    int count = 0;
    
    // 转换为HSV图像
    cv::cvtColor(image, hsvImage, cv::COLOR_BGR2HSV);
    
    // 创建一个mask   
    //cv::Scalar(26, 43, 46), cv::Scalar(34, 255, 255)
    //cv::Scalar(100, 43, 46), cv::Scalar(124, 255, 255)
    cv::inRange(hsvImage, lower, upper, mask);

    // 腐蚀和膨胀
    cv::erode(mask, mask, 2);
    cv::dilate(mask, mask, 2);

    // 找到mask中的轮廓
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::sort(contours.begin(), contours.end(), ContoursSortByArea);

    std::vector<std::vector<cv::Point>>::iterator iter = contours.begin();
    for( ; iter != contours.end() || count == 3;  iter++)
    {
        cv::minEnclosingCircle(*iter, center, radius);
        
        if(center.x < g_left.x || center.y < g_left.y || center.x > g_right.x || center.y > g_right.y)
        {
            continue;
        }
        else
        {
            RotatedRect rect = cv::minAreaRect(*iter);
            rect.points(corners);
            // for(int j=0;j<=3;j++)  
            // {  
            //     line(image,corners[j],corners[(j+1)%4],cv::Scalar(255, 0, 255),2);  
            // }
            // cv::rectangle(image, corners[0], corners[2], cv::Scalar(0, 0, 0), -1, cv::LINE_AA, 0);
            cv::circle(image, static_cast<cv::Point>(center), 25, Scalar(0, 0, 0), -1);
            count++;
        }
    }
}

cv::Point getTarget(cv::Mat outputImage)
{
    cv::Point up, down, left, right, target(-1, -1);
    if(obstacle_One.g_position.y >= obstacle_Two.g_position.y)
    {
        up = obstacle_Two.g_position;
        down = obstacle_One.g_position;
    }
    else
    {
        up = obstacle_One.g_position;
        down = obstacle_Two.g_position;
    }

    if(obstacle_One.g_position.x >= obstacle_Two.g_position.x)
    {
        left = obstacle_Two.g_position;
        right = obstacle_One.g_position;
    }
    else
    {
        left = obstacle_One.g_position;
        right = obstacle_Two.g_position;
    }
    // std::cout << down << std::endl;
    if(b_position.x <= left.x)
    {
        target.x = left.x;
        if(left.y > 180 - left.y)
        {
            target.y = left.y / 2;
            if(right.y < left.y)
            {
                target.y = (180 - left.y) / 2 + left.y;
            }
        }
        else
        {   
            target.y = (180 - left.y) / 2 + left.y;
            if(right.y > left.y)
            {
                target.y = left.y / 2;
            }
        }

        if(down.y - up.y >= 45)
        {
            target.x = (left.x + right.x) / 2;
            target.y = (left.y + right.y) / 2;
        }
    }
    else if(b_position.x > left.x && b_position.x <= right.x)
    {
        target.x = right.x;
        if(b_position.y <= 90)
        {
            target.y = right.y / 2;
        }
        else
        {
            target.y = (180 - right.y) / 2 + right.y;
        }
    }
    else
    {
        target.x = 340;
        target.y = 90;
    }
    // cv::Scalar COLOR = cv::Scalar(255, 0, 255);
    // cv::line(outputImage, ourRobotOne.position, target, COLOR, 1, cv::LINE_AA, 0);
    return target;
}

cv::Point getTarget2(cv::Mat outputImage)
{
    cv::Point target(340, -1);
    if(obstacle_Two.g_position.y - 50 > 130 - obstacle_Two.g_position.y)
    {
        target.y = 50 + (obstacle_Two.g_position.y - 50)/2;
    }
    else
    {
        target.y = 130 - (130 - obstacle_Two.g_position.y)/2;
    }
    return target;
}

int main(int argc, char* argv[])
{
    signal(SIGINT, quitFunction);

    // 初始化
    startTime = cv::getTickCount();
    b_startrecord = cv::getTickCount();
    for(auto &start : commandStartTime)
    {
        start = cv::getTickCount();
    }

    for(int i = 0; i < IMAGE_H; i++)
    {
        for(int j = 0; j < IMAGE_W; j++)
        {
            ground[i][j].x = 0;
            ground[i][j].y = 0;
        }
    }
    int xk = g_right.x - g_left.x;
    int yk = g_right.y - g_left.y;
    for(int i = 0; i < IMAGE_H; i++)
    {
        for(int j = 0; j < IMAGE_W; j++)
        {
            if(i > g_left.x || j > g_left.y || i < g_right.x || j < g_right.y)
            {
                ground[i][j].x = GOUND_H * (i - g_left.x) / xk;
                ground[i][j].y = GOUND_W * (j - 720 + g_right.y) / yk + YOFFSET;
            }
        }
    }

    int xRes = IMAGE_H;
    int yRes = IMAGE_W;
    int cameraIndex = 0;

    // 挑战赛
    int challengeIndex = 0;
    if (argc > 2) {
        xRes = std::atoi(argv[1]);
        yRes = std::atoi(argv[2]);
    }
    if (argc > 3) {
        cameraIndex = std::atoi(argv[3]);
    }
    if (argc > 4) {
        challengeIndex = std::atoi(argv[4]);
        if(challengeIndex == 0)
        {
            std::cout << "--- Dribble And Atack ---" << std::endl;
        }
        else if(challengeIndex == 1)
        {
            std::cout << "--- Free Kick ---" << std::endl;
        }
        else
        {
            std::cout << "--- DEBUG ---" << std::endl;
        }  
    }

    // The source of input images
    capture.open(cameraIndex);
    if (!capture.isOpened())
    {
        std::cerr << "Unable to initialise video capture." << std::endl;
        return 1;
    }
#ifdef OPENCV3
    capture.set(cv::CAP_PROP_FRAME_WIDTH, xRes);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, yRes);
#else
    capture.set(CV_CAP_PROP_FRAME_WIDTH, xRes);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, yRes);
#endif
    cv::Mat inputImage, convertImage;

    // The tag detection happens in the Chilitags class.
    chilitags::Chilitags chilitags;

    // The detection is not perfect, so if a tag is not detected during one frame,
    // the tag will shortly disappears, which results in flickering.
    // To address this, Chilitags "cheats" by keeping tags for n frames
    // at the same position. When tags disappear for more than 5 frames,
    // Chilitags actually removes it.
    // Here, we cancel this to show the raw detection results.
    chilitags.setFilter(0, 0.0f);

    cv::namedWindow("DisplayChilitags");

    cv::setMouseCallback("DisplayChilitags",on_mouse);
    
    cv::Scalar COLOR = cv::Scalar(255, 0, 255);

    static int status = 3;

    obstacle_One.g_position = cv::Point(250, 94);//905 400
    obstacle_Two.g_position = cv::Point(283, 54);//1016 240

    // Main loop, exiting when 'q is pressed'
    for (; 'q' != (char) cv::waitKey(1) and sRunning; ) {
        // send(0 , 7 , 2);

        // Capture a new image.
        capture.read(inputImage);
        cv::Mat outputImage = inputImage.clone();
        
        // 覆盖场地外的区域
        cv::rectangle(outputImage, cv::Point(0,0), cv::Point(1280,g_left.y), cv::Scalar(0, 0, 0), -1, cv::LINE_AA, 0);
        cv::rectangle(outputImage, cv::Point(0,0), cv::Point(g_left.x,720), cv::Scalar(0, 0, 0), -1, cv::LINE_AA, 0);
        cv::rectangle(outputImage, cv::Point(0,g_right.y), cv::Point(1280,720), cv::Scalar(0, 0, 0), -1, cv::LINE_AA, 0);
        cv::rectangle(outputImage, cv::Point(g_right.x,0), cv::Point(1280,720), cv::Scalar(0, 0, 0), -1, cv::LINE_AA, 0);

        cv::Mat ourRobotImage = outputImage.clone();
        cv::Mat oppRobotImage = outputImage.clone();

        // 覆盖蓝色
        coverIt(oppRobotImage, cv::Scalar(100, 43, 46), cv::Scalar(124, 255, 255));
        // 覆盖黄色
        coverIt(ourRobotImage, cv::Scalar(26, 43, 46), cv::Scalar(34, 255, 255));
           

        // 识别色标(传入处理后的图像)
        chilitags::TagCornerMap ourtags = chilitags.find(ourRobotImage);
        chilitags::TagCornerMap opptags = chilitags.find(oppRobotImage);

        // 识别球员
        drawTags(outputImage, ourtags, true);
        drawTags(outputImage, opptags, false);
        // cv::namedWindow("outImage");
        // cv::imshow("outImage", ourRobotImage);  

        // 识别小球
        drawBall(outputImage);

        // 画出场地矩形框
        cv::rectangle(outputImage, g_left, g_right, COLOR, 1, cv::LINE_AA, 0);

        // 画出场地中心
        cv::line(outputImage, cv::Point(0, 360), cv::Point(1280, 360), COLOR, 1, cv::LINE_AA, 0);
        cv::line(outputImage, cv::Point(640, 0), cv::Point(640, 720), COLOR, 1, cv::LINE_AA, 0);
        cv::circle(outputImage, cv::Point(640, 360), 2, Scalar(255, 0, 255), 1);

        // 画出小球和小球附近区域
        cv::circle(outputImage, static_cast<cv::Point>(b_center), (int)b_radius, Scalar(255, 0, 255), 1);
        cv::circle(outputImage, static_cast<cv::Point>(b_center), 50, Scalar(255, 0, 255), 1);

        // 标记小球的初始位置
        cv::circle(outputImage, static_cast<cv::Point>(b_init), 2, Scalar(255, 0, 255), 1);

        // 画出障碍物机器人
        cv::circle(outputImage, cv::Point(905, 367), 25, Scalar(255, 0, 255), 1);
        cv::circle(outputImage, cv::Point(1016, 240), 25, Scalar(255, 0, 255), 1);

        // 策略
        // 比赛开始等待10秒
        endTime = cv::getTickCount();
        float processingTime = ((float) endTime - startTime)/cv::getTickFrequency();
        // std::cout << processingTime << std::endl;
        // if(processingTime >= 10.0f)
        // {
            switch(challengeIndex)
            {
                case 0:
                {
                    // std::cout << "--- Dribble And Atack ---" << std::endl;

                    // 机器人与球之间的距离
                    double rob_one2ball = getDistance(ourRobotOne.g_position, b_position);

                    double rob_two2ball = getDistance(ourRobotTwo.g_position, b_position);

                    // 获取目标点
                    cv::Point ball_target = getTarget(outputImage);

                    // 机器人通讯编号
                    int robot_one_num = 0;

                    int robot_two_num = 0;

                    // 2号机器人正前方，2号机器人，球之间的夹角 - 180
                    double two_front_b_180 = getAngle(ourRobotTwo.g_position, ourRobotTwo.front, b_position) - 180.0f;

                    double two_front_b = getAngle(ourRobotTwo.g_position, ourRobotTwo.front, b_position);

                    // 2号机器人、球、目标点之间的夹角 - 180
                    double b_two_t_180 = getAngle(b_position, ourRobotTwo.g_position, ball_target) - 180.0f;
                    
                    switch(status)
                    {
                        case 0:
                        {
                            // 1号机器人踢出球
                            cv::line(outputImage, ourRobotOne.position, b_center, COLOR, 1, cv::LINE_AA, 0);

                            // 机器人正对球的角度 - 180
                            double one_front_b =  getAngle(ourRobotOne.g_position, ourRobotOne.front, b_position) - 180.0f;

                            std::cout << "阶段：" << status  << "  " << "1号机器人：" <<  one_front_b  << "  " << rob_one2ball << "  " << isBallMove() << std::endl;
                            if(!isBallMove())
                            {
                                if(one_front_b > -175.0f && one_front_b <= 0)
                                {
                                    std::cout << "turn right" << std::endl;
                                    send(robot_one_num , 4 , 2);          
                                }
                                else if(one_front_b < 175.0f && one_front_b >= 0)
                                {
                                    std::cout << "turn left" << std::endl;
                                    send(robot_one_num , 3 , 2); 
                                }
                                else
                                {
                                    std::cout << "go ahead" << std::endl;
                                    send(robot_one_num , 11 , 2);
                                }
                            }
                            else
                            {
                                status = 1;
                            }  
                            break;
                        }
                        case 1:
                        {
                            // 1号机器人后退
                            if(rob_one2ball < 50)
                            {
                                send(robot_one_num , 18 , 2);
                            }
                            else
                            {
                                std::cout << "1号机器人停止" << std::endl;
                                send(robot_one_num , 14 , 2);
                            }

                            // 2号机器人正对球
                            cv::line(outputImage, ourRobotTwo.position, b_center, COLOR, 1, cv::LINE_AA, 0);

                            std::cout << "阶段：" << status << "  " << "2号机器人：" <<  two_front_b_180  << "  " << b_two_t_180 << "  " << two_front_b << "  " << rob_two2ball << "  " << ball_target << std::endl;
                            
                            if(rob_two2ball >= 20)
                            {
                                if(two_front_b_180 > -175.0f && two_front_b_180 <= 0)
                                {
                                    std::cout << "turn right" << std::endl;
                                    // send(robot_two_num , 4 , 2);
                                    send(robot_two_num , 6 , 1);          
                                }
                                else if(two_front_b_180 < 175.0f && two_front_b_180 >= 0)
                                {
                                    std::cout << "turn left" << std::endl;
                                    // send(robot_two_num , 3 , 2);
                                    send(robot_two_num , 5 , 1); 
                                }
                                else
                                {
                                    std::cout << "go ahead" << std::endl;
                                    // send(robot_two_num , 11 , 2);
                                    send(robot_two_num , 1 , 1);
                                }
                            }
                            else
                            {
                                status = 2;                              
                            }
                            break;
                        }
                        case 2:
                        {
                            // 策略1：2号机器人调整方向，正前方正对目标点
                            cv::line(outputImage, ourRobotTwo.position, b_center, COLOR, 1, cv::LINE_AA, 0);

                            std::cout << "阶段：" << status << "  " << "2号机器人：" <<  two_front_b_180  << "  " << b_two_t_180 << "  " << two_front_b << "  " << rob_two2ball << "  " << ball_target << std::endl;

                            if(rob_two2ball < 20)
                            {
                                if(b_two_t_180 >= -180.0f && b_two_t_180 <= -5.0f)
                                {
                                    std::cout << "move right" << std::endl;
                                    // send(robot_two_num , 6 , 2);
                                    send(robot_two_num , 9 , 1);
                                }
                                else if(b_two_t_180 <= 180.0f && b_two_t_180 >= 5.0f)
                                {
                                    std::cout << "move left" << std::endl;
                                    // send(robot_two_num , 5 , 2);
                                    send(robot_two_num , 7 , 1);
                                }
                                else
                                {
                                    // 正对球
                                    if(two_front_b_180 > -175.0f && two_front_b_180 <= 0)
                                    {
                                        std::cout << "--- turn right" << std::endl;
                                        // send(robot_two_num , 4 , 2);
                                        send(robot_two_num , 6 , 1);          
                                    }
                                    else if(two_front_b_180 < 175.0f && two_front_b_180 >= 0)
                                    {
                                        std::cout << "--- turn left" << std::endl;
                                        // send(robot_two_num , 3 , 2);
                                        send(robot_two_num , 5 , 1); 
                                    }
                                    else
                                    {
                                        // send(robot_two_num , 11 , 2);
                                        send(robot_two_num , 1 , 1);
                                    }
                                }
                            }
                            else
                            {
                                status = 1;
                            }
                            
                            break;
                        }
                        case 3:
                        {
                            // 策略2：2号机器人调整方向，转动到踢球角度
                            double b_target_right =  getAngle(b_position, ball_target, cv::Point(b_position.x + 20, b_position.y));  
                            double r_front_right =  getAngle(ourRobotTwo.g_position, ourRobotTwo.front, cv::Point(ourRobotTwo.position.x + 20, ourRobotTwo.position.y));
                            double t_angle = 0.0f;  

                            if(b_target_right < 180)
                            {
                                t_angle = 90 + b_target_right;
                            }
                            else
                            {
                                t_angle = b_target_right - 270;
                            }

                            double angle_d = r_front_right - t_angle;
                            if(angle_d > 3 && angle_d < 180)
                            {
                                std::cout << "--- turn right" << std::endl;
                                // send(robot_two_num , 4 , 2);
                                send(robot_two_num , 6 , 1);
                            }
                            else if(angle_d < 360 - t_angle && angle_d > 180)
                            {
                                std::cout << "--- turn left" << std::endl;
                                // send(robot_two_num , 3 , 2);
                                send(robot_two_num , 5 , 1); 
                            }
                            else if(angle_d < -3 && angle_d > -t_angle)
                            {
                                std::cout << "--- turn left" << std::endl;
                                // send(robot_two_num , 3 , 2);
                                send(robot_two_num , 5 , 1);
                            }
                            else
                            {
                                std::cout << "--- stop" << std::endl;
                                // send(robot_two_num , 0 , 1); 
                                status = 5;
                            }

                            std::cout << "阶段：" << status << "  " << "2号机器人：" << r_front_right << "  " << t_angle << std::endl;

                            break;
                        }
                        case 4:
                        {
                            // 策略2：2号机器人调整位置，移动到踢球位置

                            // double b_target_right =  getAngle(b_position, ball_target, cv::Point(b_position.x + 20, b_position.y));
                            double r_front_ball =  getAngle(ourRobotTwo.g_position, ourRobotTwo.front, ball_target);

                            std::cout << "阶段：" << status << "  " << "2号机器人：" << r_front_ball << std::endl;

                            if(r_front_ball < 85)
                            {
                                std::cout << "go go go" << std::endl;
                                // send(robot_two_num , 11 , 2);
                                send(robot_two_num , 1 , 1);
                            }
                            else if(r_front_ball > 95)
                            {
                                std::cout << "back back" << std::endl;
                                // send(robot_two_num , 11 , 2);
                                send(robot_two_num , 2 , 1);
                            }
                            else
                            {
                                std::cout << "--- stop" << std::endl;
                                // send(robot_two_num , 0 , 1); 
                                status = 5;
                            }
                            break;
                        }
                        case 5:
                        {

                        }
                        default:
                        {
                            // std::cout << "gogogogo!!!" << std::endl;
                            // send(0 , 15 , 2);
                        }
                    }
                    
                    break;
                    
                }
                case 1:
                {
                    std::cout << "--- Free Kick ---" << std::endl;
                    // 获取目标点
                    cv::Point ball_target = getTarget2(outputImage);

                    // 2号机器人正前方，2号机器人，球之间的夹角 - 180
                    double two_front_b_180 = getAngle(ourRobotTwo.g_position, ourRobotTwo.front, b_position) - 180.0f;

                    double two_front_b = getAngle(ourRobotTwo.g_position, ourRobotTwo.front, b_position);

                    // 2号机器人、球、目标点之间的夹角 - 180
                    double b_two_t_180 = getAngle(b_position, ourRobotTwo.g_position, ball_target) - 180.0f;

                    // 机器人与球之间的距离
                    double rob_two2ball = getDistance(ourRobotTwo.g_position, b_position);

                    // std::cout << ball_target << std::endl;  

                    // 机器人通讯编号
                    int robot_two_num = 0;

                    switch(status)
                    {
                        case 0:
                        {
                            // 2号机器人正对球
                            cv::line(outputImage, ourRobotTwo.position, b_center, COLOR, 1, cv::LINE_AA, 0);

                            std::cout << "阶段：" << status << "  " << "2号机器人：" <<  two_front_b_180  << "  " << b_two_t_180 << "  " << two_front_b << "  " << rob_two2ball << "  " << ball_target << std::endl;

                            if(rob_two2ball >= 20)
                            {
                                if(two_front_b_180 > -175.0f && two_front_b_180 <= 0)
                                {
                                    std::cout << "turn right" << std::endl;
                                    // send(robot_two_num , 4 , 2);
                                    send(robot_two_num , 6 , 1);          
                                }
                                else if(two_front_b_180 < 175.0f && two_front_b_180 >= 0)
                                {
                                    std::cout << "turn left" << std::endl;
                                    // send(robot_two_num , 3 , 2);
                                    send(robot_two_num , 5 , 1); 
                                }
                                else
                                {
                                    std::cout << "go ahead" << std::endl;
                                    // send(robot_two_num , 11 , 2);
                                    send(robot_two_num , 1 , 1);
                                }
                            }
                            else
                            {
                                status = 1;                              
                            }
                            break;
                        }
                        case 1:
                        {
                            // 策略2：2号机器人调整方向，转动到踢球角度
                            double b_target_right =  getAngle(b_position, ball_target, cv::Point(b_position.x + 20, b_position.y));  
                            double r_front_right =  getAngle(ourRobotTwo.g_position, ourRobotTwo.front, cv::Point(ourRobotTwo.position.x + 20, ourRobotTwo.position.y));
                            double t_angle = 0.0f;  

                            if(b_target_right < 180)
                            {
                                t_angle = 90 + b_target_right;
                            }
                            else
                            {
                                t_angle = b_target_right - 270;
                            }

                            double angle_d = r_front_right - t_angle;
                            if(angle_d > 3 && angle_d < 180)
                            {
                                std::cout << "--- turn right" << std::endl;
                                // send(robot_two_num , 4 , 2);
                                send(robot_two_num , 6 , 1);
                            }
                            else if(angle_d < 360 - t_angle && angle_d > 180)
                            {
                                std::cout << "--- turn left" << std::endl;
                                // send(robot_two_num , 3 , 2);
                                send(robot_two_num , 5 , 1); 
                            }
                            else if(angle_d < -3 && angle_d > -t_angle)
                            {
                                std::cout << "--- turn left" << std::endl;
                                // send(robot_two_num , 3 , 2);
                                send(robot_two_num , 5 , 1);
                            }
                            else
                            {
                                std::cout << "--- stop" << std::endl;
                                // send(robot_two_num , 0 , 1); 
                                status = 2;
                            }

                            std::cout << "阶段：" << status << "  " << "2号机器人：" << r_front_right << "  " << t_angle << std::endl;
                            break;
                        }
                        case 2:
                        {
                            
                            break;
                        }
                        case 3:
                        {
                            break;
                        }
                        default:
                        {

                        }
                    }
                    break;
                }
                default:
                {
                    std::cout << "visual test" << std::endl;
                }
            }
        

        // Finally...
        cv::imshow("DisplayChilitags", outputImage);
    }
    
    cv::destroyWindow("DisplayChilitags");
    capture.release();

    return 0;
}
