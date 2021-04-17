#include "control.h"
#include "configure.h"

// using namespace cv;
// using namespace std;

struct Armor_data
{
    RotatedRect armor_rect;///
    float width = 0; ///装甲板宽度
    float height = 0; ///装甲板高度
    float aspect_ratio = 0; ///装甲板宽高比
    float tan_angle = 0;///
    RotatedRect left_light;///
    RotatedRect right_light;///
    
    float left_light_width = 0; ///左灯条宽度
    float right_light_width = 0; ///右灯条高度

    float left_light_height = 0; ///左灯条高度
    float right_light_height = 0; ///右灯条高度
    
    float left_right_h = 0;///左灯条高和右灯条高比值
    float left_right_w = 0;///左灯条宽和右灯条宽比值
    
    float min_light_h = 0;//最小灯条高度
    float max_light_h = 0;//最大灯条高度
    int depth = 0; //装甲板深度
    
    int priority = 0;//优先级 多装甲板情况

    int distinguish = 0;///大小装甲板 小0 大1
    int position = 0;//装甲板在车的左(-1)右(1)位置
};

/**
 * @brief 图像处理
 * 
 */
class ImageProcess
{
public:
    void Pretreat_Rgb(Mat frame, int enemy_color);
    bool Processing();
    bool light_Judge(int i, int j);//判断左右灯条能否组成装甲板
    int average_Color();//计算图像颜色平均值
    void fitting_Armor();//拟合装甲板
    void find_Light();//寻找灯条
    void armor_Screening();//筛选装甲板
    void free_Memory();//释放内存
    int motion_Direction();//判断装甲板运动方向
    void roi_Range();//ROI范围
    // void direction_Judgment();//方向判断
    
    void speed_Calculation();//自动开火
    void pretreat_Hsv(Mat src_img, int enemy_color);//hsv预处理
    ImageProcess() {}
    ~ImageProcess() {}
    
    Mat frame;//原图
    Mat mask;//二值化
    Mat gray_img;//灰度图
    Mat draw_img;//画板 
    
    Armor_data armor_data;
    
    vector<Armor_data> armor;
    vector<RotatedRect> light;

    Rect armor_roi;

    Point lost_armor_center;
    Point armor_center;//装甲板中心点
    
    bool lost_armor_success = false;
    bool armor_success = false;
    bool switch_armor = false;//切换装甲板

    int lost_distance_armor = 0;//两帧装甲板之间的距离
    int amplitude = 0;//幅度
    int armor_count = 0;//装甲板计数
    int light_count = 0;//灯条计数
    int optimal_armor = 0;//最优装甲板
    int armor_position = 0;//装甲板在车的位置
    int armor_direction = 0;//1向右 -1 向左
    int num = 0;//运行次数
    //蓝色th参数
    int blue_armor_gray_th = 50;
    int blue_armor_color_th = 70;
    //红色th参数
    int red_armor_gray_th = 11; //60 45
    int red_armor_color_th = 11;//192 95 10.50
    // Point lost_armor_center;//上一帧装甲板位置

    int roi_num = 0;//roi计数
    int lose_roi_num = 0;//roi丢失计数
    int roi_temp = 0;
    int roi_num_law[5] = {0};//roi规律
    int armor_gray_th = 100;
    int h_min = 0;//0 80 116 222 21 95 red
    int h_max = 80;//115 173 136 255 90 255 blue
    int s_min = 0;
    int s_max = 255;
    int v_min = 140;
    int v_max = 255;
};


