#include "configure.h"
#include "control.h"
#include "armor/armorplate.h"
#include "pnp/solvepnp.h"
#include "serial/serialport.h"
#include "camera/videocapture.h"
#include "kalmantest/kalmantest.h"
#include "detect_buff/buff_detect.h"
class WorKing
{
public:
    WorKing();
    ~WorKing();
    void Run();              //主函数
    void ddd();              //拍摄图片
    void Run_MAX_Talisman(); //大神符
    
    void Return_zero();//归零
    void Angle_compensate();//pnp计算角度补偿
    void Mode_Selection(); // 模式选择

    ImageProcess img;
    Max_Buff buff;
    SerialPort serial;
    SolveP4p pnp;
    VideoCapture capture;
    VideoCap cap;
    RM_kalmanfilter kalman;
    
    
    Mat frame;//保留原图
    // Mat src_img;//ROI 图像
    
    int enemy_color = 1;//敌方颜色
    int pattern = 0;//模式选择
    bool data_success = false;
    int _yaw = 0;
    float yaw = 0;
    int _pitch = 0;
    float pitch = 0;
    int depth = 0;
    int data_type = 0;
    int is_shooting = 0;

    int offset_x = 100;
    int offset_y = 200;
    int _offset_x = 1;
    int _offset_y = 0;
};
