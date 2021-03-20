#include "armorplate.h"

/**
 * @brief 求两点之间的距离
 * 
 * @param a 点A
 * @param b 点B
 * @return double 两点之间的距离 
 */
float Distance(Point a, Point b)
{
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}
/**
 * @brief 释放内存
 * 
 */
void ImageProcess::Free_memory()
{
    lost_armor_success = armor_success;
    armor_success = false;
    if(light_count > 0)
    {
        light.clear();
        vector<RotatedRect> (light).swap(light);
        if(armor_count > 0)
        {
            armor.clear();
            vector<Armor_data> (armor).swap(armor);
        }
    }
}

/**
 * @brief 图像预处理
 * 
 * @param src_img -传入原图像
 * @param enemy_color -传入敌方颜色
 */
void ImageProcess::Pretreat(Mat src_img, int enemy_color)
{
    //保存原图像
    this->frame = src_img;
    // imshow(" ", frame);
    //转灰度图
    Mat gray_img;
    cvtColor(this->frame, gray_img, COLOR_BGR2GRAY);
    //分离通道
    vector<Mat> _split;
    split(this->frame, _split);
    //判断颜色
    Mat bin_img_color, bin_img_gray;
    // cout<<enemy_color<<endl;
    if (enemy_color == 0)
    {
        subtract(_split[0], _split[2], bin_img_color); // b - r
#if IS_PARAM_ADJUSTMENT == 1
        namedWindow("src_img", WINDOW_AUTOSIZE);
        createTrackbar("GRAY_TH_BLUE:", "src_img", &blue_armor_gray_th, 255, NULL);
        createTrackbar("COLOR_TH_BLUE:", "src_img", &blue_armor_color_th, 255, NULL);
        threshold(gray_img, bin_img_gray, this->blue_armor_gray_th, 255, THRESH_BINARY);
        threshold(bin_img_color, bin_img_color, this->blue_armor_color_th, 255, THRESH_BINARY);
#else
        threshold(gray_img, bin_img_gray, blue_armor_gray_th, 255, THRESH_BINARY);
        threshold(bin_img_color, bin_img_color, blue_armor_color_th, 255, THRESH_BINARY);
#endif
    }
    else if (enemy_color == 1)
    {
        subtract(_split[2], _split[0], bin_img_color); // r - b
#if IS_PARAM_ADJUSTMENT == 1
        namedWindow("src_img", WINDOW_AUTOSIZE);
        createTrackbar("GRAY_TH_RED:", "src_img", &red_armor_gray_th, 255);
        createTrackbar("COLOR_TH_RED:", "src_img", &red_armor_color_th, 255);
        threshold(gray_img, bin_img_gray, this->red_armor_gray_th, 255, THRESH_BINARY);
        threshold(bin_img_color, bin_img_color, this->red_armor_color_th, 255, THRESH_BINARY);
#else
        threshold(gray_img, bin_img_gray, red_armor_gray_th, 255, THRESH_BINARY);
        threshold(bin_img_color, bin_img_color, red_armor_color_th, 255, THRESH_BINARY);
#endif                              //输出帧率
    }

    Mat element = getStructuringElement(MORPH_ELLIPSE, cv::Size(3, 7));

    dilate(bin_img_gray, bin_img_gray, element);
    dilate(bin_img_color, bin_img_color, element);
    bitwise_and(bin_img_color, bin_img_gray, bin_img_color);
    dilate(bin_img_color, bin_img_color, element);
#if SHOW_BIN_IMG == 1
    imshow("gray_img", bin_img_gray);
    imshow("mask", bin_img_color);
    imshow("src_img", bin_img_color);
#endif
    //保存处理后的图片
    this->mask = bin_img_color;
    this->gray_img = bin_img_gray;
    
    //清空内存
    _split.clear();
    vector<Mat>(_split).swap(_split);
}

/**
 * @brief 寻找灯条
 * 
 */
void ImageProcess::Find_light()
{
    RotatedRect box;
    /*轮廓周长*/
    int perimeter = 0;
    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    //筛选，去除一部分矩形
    for (size_t i = 0; i < contours.size(); i++)
    {
        perimeter = arcLength(contours[i], true); 
        //轮廓周长
        if (perimeter < 200 && perimeter > 4000 && contours[i].size() < 5 && contourArea(contours[i]) < 5000)
        {
            continue;
        }
        
        //椭圆拟合
        box = fitEllipse(Mat(contours[i]));

        //重新定义长宽和角度
        if (box.angle > 90.0f)
        {
            box.angle = box.angle - 180.0f;
        }
        
        //灯条长宽比
        float light_h_w;
        float  _h = MAX(box.size.width, box.size.height);
        float  _w = MIN(box.size.width, box.size.height);
        light_h_w = _w / _h;
        // cout <<light_h_w<<endl;
        if (fabs(box.angle) < 40 && light_h_w < 0.4f)
        {
            this->light.push_back(box); //保存灯条
            light_count ++;
#if DRAW_LIGHT_IMG == 1
            Point2f vertex[4];
            box.points(vertex);
            for (int l = 0; l < 4; l++)
            {
                line(draw_img, vertex[l], vertex[(l + 1) % 4], Scalar(0, 255, 255), 3, 8);
            }
            imshow("draw", draw_img);
#endif
        }
    }
    //清空内存
    contours.clear();
    vector<vector<Point>> (contours).swap(contours);
}
/**
 * @brief 数据处理
 * 
 * @return true 正确得到装甲板
 * @return false 无法得到装甲板
 */
bool ImageProcess::Processing()
{
    //归零
    armor_count = 0;
    draw_img = Mat::zeros(frame.size(), CV_8UC3);
    light_count = 0;
    //寻找灯条
    Find_light();
    
    //灯条少于2 停止运行
    if(this->light_count < 2)
    {
        return false;
    }
    
    //灯条拟合装甲板
    Fitting_armor();
    
    //无装甲
    if(armor_count < 1)
    {
        return false;
    }
    //
    armor_success = true;

    //多装甲板筛选
    Armor_screening();
    
    return true;
}


/**
 * @brief 多装甲板筛选
 * 
 */
void ImageProcess::Armor_screening()
{
    if(armor_count < 2)
    {
        optimal_armor = 0;
    }
    else
    {
        int max_priority = 0;
        for (int i = 0; i < armor_count; i ++)
        {
            
            //判断左右两个灯条y点差值
            int diff_y = fabs(armor[i].left_light.center.y - armor[i].right_light.center.y);
            if(diff_y < armor[i].min_light_h * 0.15)
            {
                armor[i].priority += 3;
            }
            else if(diff_y < armor[i].min_light_h *0.25)
            {
                armor[i].priority +=2;
            }
            else if(diff_y < armor[i].min_light_h *0.5)
            {
                armor[i].priority +=1;
            }

            //判断左右两点高度差
            int diff_h = fabs(armor[i].left_light_height - armor[i].right_light_height);
            if(diff_h < armor[i].max_light_h * 0.1)
            {
                armor[i].priority +=2;
            }
            else if(diff_h < armor[i].min_light_h * 0.2)
            {
                armor[i].priority +=1;
            }
            
            //判断装甲板在车的左右
            cout<<armor[i].tan_angle<<endl;
            if(armor[i].tan_angle <= 5 && armor[i].tan_angle >= -5)//中
            {
                armor[i].priority +=2;
                armor[i].position = 0;
            }
            else if(armor[i].tan_angle > 5)//右
            {
                armor[i].priority +=1;
                armor[i].position = 1;
            }
            else//左
            {
                armor[i].priority +=1;
                armor[i].position = 1;
            }

            if(armor[i].left_right_h < 1.2 || armor[i].left_right_h > 0.8)
            {
                armor[i].priority +=1;
            }
            if(armor[i].left_right_w < 1.2 || armor[i].left_right_w > 0.8)
            {
                armor[i].priority +=1; 
            }
            //优先小装甲板
            if(armor[i].distinguish == 0)
            {
                armor[i].priority +=1;
            }

            if(armor[i].priority > max_priority)
            {
                optimal_armor = i;
            }
            else if(armor[i].priority == max_priority)//优先级相同比较高度
            {
                if(fabs(armor[i].tan_angle) < fabs(armor[optimal_armor].tan_angle))
                {
                    optimal_armor = i;
                }
                else if(armor[i].height > armor[optimal_armor].height)
                {
                    optimal_armor = i;
                }
            }
        }
    }
#if DRAW_ARMOR_IMG == 1
    rectangle(draw_img, armor[optimal_armor].armor_rect.boundingRect(), Scalar(0, 0, 255), 3, 8);
    imshow("draw", draw_img);
    draw_img = Mat::zeros(frame.size(), CV_8UC3);
#endif
    armor_position = armor[optimal_armor].position;
    armor_center = armor[optimal_armor].armor_rect.center;
}
/**
 * @brief 拟合装甲板
 * 
 */
void ImageProcess::Fitting_armor()
{
     //遍历灯条
    for (size_t i = 0; i < this->light.size(); i++)
    {
        for (size_t j = i + 1; j < this->light.size(); j++)
        {
            //区分左右灯条
            int light_left = 0, light_right = 0;
            if (this->light[i].center.x > this->light[j].center.x)
            {
                light_left = j;
                light_right = i;
            }
            else
            {
                light_left = i;
                light_right = j;
            }
    
            //保存左右灯条
            armor_data.left_light = this->light[light_left];
            armor_data.right_light = this->light[light_right];
            //计算灯条中心点形成的斜率
            float error_angle = atan((light[light_right].center.y - light[light_left].center.y) / (light[light_right].center.x - light[light_left].center.x));
            // cout<<error_angle<<endl;
            if (error_angle < 5.0f)
            {
                armor_data.tan_angle = atan(error_angle) * 180 / CV_PI;
                if (this->Light_judge(light_left, light_right))
                {   
                    if (this->Average_color() < 20)
                    {
                        armor_count++;
                        armor.push_back(armor_data);
                        // armor_count++;//数据正确则保存
                    }
                }
            }
        }
    }
}

/**
 * @brief 寻找可能为装甲板的位置
 * @param i 左light的下标
 * @param j 右light的下标
 * @return true 找到了符合装甲板条件的位置
 * @return false 没找到了符合装甲板条件的位置
 */
bool ImageProcess::Light_judge(int i, int j)
{
    armor_data.left_light_height = MAX(light[i].size.height, light[i].size.width);
    armor_data.left_light_width = MIN(light[i].size.height, light[i].size.width);
    armor_data.right_light_height = MAX(light[j].size.height, light[j].size.width);
    armor_data.right_light_width = MIN(light[j].size.height, light[j].size.width);
    armor_data.min_light_h = MIN(armor_data.right_light_height, armor_data.left_light_height);
    armor_data.max_light_h = MAX(armor_data.right_light_height, armor_data.left_light_height);
    armor_data.left_right_h = armor_data.left_light_height / armor_data.right_light_height;
    armor_data.left_right_w = armor_data.left_light_width / armor_data.right_light_width;
    if (armor_data.left_right_h < 1.2 
            && armor_data.left_right_w > 0.2 
            && armor_data.left_right_h > 0.8
            && armor_data.left_right_w < 2)
    {
        armor_data.height = (armor_data.right_light_height + armor_data.left_light_height) / 2.0f;
        // 两个灯条高度差不大
        if (fabs(armor_data.left_light.center.y - armor_data.right_light.center.y) < armor_data.height / 2)
        {
            if(fabs(armor_data.left_light_height  - armor_data.right_light_height) < armor_data.height / 4)
            {
                armor_data.width = Distance(armor_data.left_light.center, armor_data.right_light.center);
                armor_data.aspect_ratio = armor_data.width/armor_data.height;//保存长宽比
                // cout<<armor_data.left_light.angle - armor_data.right_light.angle<<endl;
                if(armor_data.left_light.angle - armor_data.right_light.angle < 8)//两侧灯条角度差
                {
                    //装甲板长宽比
                    if (armor_data.aspect_ratio < 2.5f && armor_data.aspect_ratio > 0.5f)
                    {
                        armor_data.distinguish = 0;//小装甲板
                        return true;
                    }
                    
                    if (armor_data.aspect_ratio > 3.05f && armor_data.aspect_ratio < 4.1f)
                    {
                        armor_data.distinguish = 1;//大装甲板
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

/**
 * @brief 计算图像中像素点的平均强度
 * 
 * @param roi 传入需要计算的图像
 * @return int 返回平均强度
 */
int ImageProcess::Average_color()
{   
    RotatedRect rects = RotatedRect((armor_data.left_light.center + armor_data.right_light.center)/2,
        Size(armor_data.width - (armor_data.left_light_width + armor_data.right_light_width)/2, 
        (armor_data.left_light_height + armor_data.right_light_height)/2),
        armor_data.tan_angle);
    armor_data.armor_rect = rects; //储存装甲板旋转矩形
    Rect _rect = rects.boundingRect();
    if (_rect.x <= 0)
    {
        _rect.x = 0;
    }
    if (_rect.y <= 0)
    {
        _rect.y = 0;
    }
    if (_rect.y + _rect.height >= gray_img.rows)
    {
        _rect.height = gray_img.rows - _rect.y;
    }
    if (_rect.x + _rect.width >= gray_img.cols)
    {
        _rect.width = gray_img.cols - _rect.x;
    }
    Mat roi = gray_img(_rect);;
    int average_intensity = static_cast<int>(mean(roi).val[0]);
    cout<<"average_intensity = "<<average_intensity<<endl;
    return average_intensity;
}


void ImageProcess::Roi_Range()
{
        if (lost_armor_success)
        {
          int point_x = armor[optimal_armor].armor_rect.center.x - 240 + armor_roi.x;
          int point_y = armor[optimal_armor].armor_rect.center.y - 150 + armor_roi.y;
          int width = 480;
          int height = 300;

            if (point_x < 0)
            {
              point_x = 0;
            }
            if (point_y < 0)
            {
              point_y = 0;
            }
            if (point_x + width >= CAMERA_RESOLUTION_COLS)
            {
              width = CAMERA_RESOLUTION_COLS - abs(point_x);
            }
            if (point_y + height >= CAMERA_RESOLUTION_ROWS)
            {
              height = CAMERA_RESOLUTION_ROWS - abs(point_y);
            }
            armor_roi = Rect(point_x, point_y, width, height);
            // armor[optimal_armor].armor_rect = RotatedRect(
                // Point(armor[optimal_armor].armor_rect.center.x + armor_roi.x,
                    //   armor[optimal_armor].armor_rect.center.y + armor_roi.y),
                // Size(armor[optimal_armor].width, armor[optimal_armor].height), armor[optimal_armor].tan_angle);
            // rectangle(frame, armor_roi, Scalar(0, 255, 255), 3, 8);
        }
        else
        {
          int point_x = armor[optimal_armor].armor_rect.center.x - 240;
          int point_y = armor[optimal_armor].armor_rect.center.y - 150;
          int width = 480;
          int height = 300;
          if (point_x < 0)
          {
            point_x = 0;
          }
          if (point_y < 0)
          {
            point_y = 0;
          }
          if (point_x + width >= CAMERA_RESOLUTION_COLS)
          {
            width = CAMERA_RESOLUTION_COLS - point_x;
          }
          if (point_y + height >= CAMERA_RESOLUTION_ROWS)
          {
            height = CAMERA_RESOLUTION_ROWS - point_y;
          }
          armor_roi = Rect(point_x, point_y, width, height);
        }
}