#include "configure.h"
#include "control.h"
#include "armor/armorplate.h"
#include "pnp/solvepnp.h"
#include "serial/serialport.h"
#include "camera/videocapture.h"
#include "kalmantest/kalmantest.h"
#include "detect_buff/buff_detect.h"
#include "stdlib.h"
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
    void Automatic_fire();//自动开火
    bool top();//小陀螺判断(陀螺仪)
    ImageProcess img;
    Max_Buff buff;
    SerialPort serial;
    SolveP4p pnp;
    VideoCapture capture;
    VideoCap cap;
    // RM_kalmanfilter kalman;
    
    
    Mat frame;//保留原图
    // Mat src_img;//ROI 图像
    
    int enemy_color = 1;//敌方颜色
    int pattern = 0;//模式选择
    int firing = 0;//射速15 18 30
    int fire_num = 0;//开火计数
    bool data_success = false;
    bool judge_top = false;//小陀螺或者扭腰判断
    int gyro_arr[50] = {0};//陀螺仪数据储存 50帧分析一次
    int _yaw = 0;
    float yaw = 0;
    int _pitch = 0;
    float pitch = 0;
    int depth = 0;
    int data_type = 0;
    int is_shooting = 0;

    int offset_x = 670;
    int offset_y = 100;
    int _offset_x = 1;
    int _offset_y = 0;

    float X_last; 
    float X_mid;  
    float X_now;  
    float P_mid;  
    float P_now;  
    float P_last; 
    float kg;     //kalman增益
    float A;  
    float Q;
    float R;
    float H;
    
    void kalmanCreate(float T_Q,float T_R)
    {
        
        X_last = (float)0;
        P_last = 0;
        Q = T_Q;
        R = T_R;
        A = 1;
        H = 1;
        X_mid = X_last;
    }


    float KalmanFilter(float dat)
    {
        X_mid =A*X_last;                    
        P_mid = A*P_last+Q;               
        kg = P_mid/(P_mid+R);            
        X_now = X_mid+kg*(dat-X_mid);     
        P_now = (1-kg)*P_mid;            
        P_last = P_now;                         //状态更新
        X_last = X_now;
        return X_now;
    }
};




