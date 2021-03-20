#include "control.h"
#include "configure.h"

using namespace cv;
using namespace std;

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
    void Pretreat(Mat frame, int enemy_color);
    bool Processing();
    bool Light_judge(int i, int j);//判断左右灯条能否组成装甲板
    int Average_color();//计算图像颜色平均值
    void Fitting_armor();//拟合装甲板
    void Find_light();//寻找灯条
    void Armor_screening();//筛选装甲板
    void Free_memory();//释放内存
    int Motion_direction();//判断装甲板运动方向
    void Roi_Range();//ROI范围
    
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

    bool lost_armor_success = false;
    bool armor_success = false;
    int armor_count = 0;//装甲板计数
    int light_count = 0;//灯条计数
    int optimal_armor = 0;//最优装甲板
    int armor_position = 0;//装甲板在车的位置
    int armor_direction = 0;//1向右 -1 向左
    Point armor_center;//装甲板中心点
    int num = 0;//运行次数
    //蓝色th参数
    int blue_armor_gray_th = 70;
    int blue_armor_color_th = 70;
    //红色th参数
    int red_armor_gray_th = 30; //视频20
    int red_armor_color_th = 45;


};


