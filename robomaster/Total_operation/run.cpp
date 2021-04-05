#include "run.h"

WorKing::WorKing()
    : capture(USB_CAPTURE_DEFULT), cap(ISOPEN_INDUSTRY_CAPTURE) {}

WorKing::~WorKing() {}
/**
 * @brief 运行函数
 * 
 */
float offset_yaw = 1;
    
void WorKing::Run()
{
#if IS_PARAM_ADJUSTMENT == 1
    namedWindow("parameter", WINDOW_AUTOSIZE);
    createTrackbar("offset_x:", "parameter", &offset_x, 2000, NULL);
    createTrackbar("offset_y:", "parameter", &offset_y, 2000, NULL);
    createTrackbar("_offset_x:", "parameter", &_offset_x, 1, NULL);
    createTrackbar("_offset_y:", "parameter", &_offset_y, 1, NULL);

#endif
    for (;;)
    {
#if FPS_SHOW == 1
        double t = (double)cv::getTickCount(); //开始计时
#endif                                     // #endif
        if (cap.isindustryimgInput())
        {
            frame = cvarrToMat(cap.iplImage, true);
        }
        else
        {
            capture >> frame;
        }

        
        Mode_Selection();

#if ISOPEN_INDUSTRY_CAPTURE == 1
        resize(frame, frame, Size(CAMERA_RESOLUTION_COLS, CAMERA_RESOLUTION_ROWS));
#endif
        Mat src_img;
#if ROI_IMG == 1
        // ROI
        if (img.lost_armor_success)
        {
            src_img = frame(img.armor_roi);
            // imshow("roi", src_img);
        }
        else
        {
            src_img = frame; //直接赋值
            img.armor_roi = Rect(0, 0, 0, 0);
        }
#elif ROI_IMG == 0
    src_img = frame;
#endif
        pattern = 0;
        enemy_color = 1;
        switch (this->pattern)
        {
        case 0://自瞄
            img.num++;
            // img.Pretreat(src_img, enemy_color);
            img.pretreat_Hsv(src_img, enemy_color);
            data_success = img.Processing();
            if(data_success)
            {
                data_type = 1;
                Point2f roi_tl = Point2f(0, 0);
                if(img.lost_armor_success)
                {
                    roi_tl = img.armor_roi.tl();
                }
#if CALL_KALMAN == 1
            data_type = 1;
            Point kalman_point = kalman.predict_point(t,img.armor[img.optimal_armor].armor_rect.center + roi_tl);
            // circle(frame, kalman_point, 10, Scalar(0, 0, 255), -1);
            // if(kalman_point.x > img.armor[img.optimal_armor].armor_rect.center.x + roi_tl.x)
            // {
            //   offset_x = 800;
            // }
            // else if(kalman_point.x < img.armor[img.optimal_armor].armor_rect.center.x + roi_tl.x) 
            // {
            //   offset_x = 800;
            // }
            // else{
            //   offset_x = 100;
            // }

#endif

#if CALL_DEPTH_INFORMATION == 1
            RotatedRect box = RotatedRect(img.armor[img.optimal_armor].armor_rect.center + roi_tl, 
                Size(img.armor[img.optimal_armor].width, img.armor[img.optimal_armor].height), 
                img.armor[img.optimal_armor].tan_angle);
            pnp.vertex_Sort(box);
            rectangle(frame, box.boundingRect(), Scalar(0, 255, 0), 3, 8);
            
            //pnp角度结算
            if(img.armor[img.optimal_armor].distinguish == 0)
            {
                pnp.run_SolvePnp(SMALL_ARMORPLATE_WIDTH, ARMORPLATE_HIGHT);
            }
            else
            {
                pnp.run_SolvePnp(BIG_ARMORPLATE_WIDTH, ARMORPLATE_HIGHT);
            }
            yaw = pnp.angle_x;
            pitch = pnp.angle_y;
            depth = int(pnp.dist);

            //补偿角度
            Angle_compensate();
            
            //自动开火
            Automatic_fire();
#endif
            //ROI范围
#if ROI_IMG == 1
            img.roi_Range();
#endif
        }
        else//丢失清零
        { 
            img.lost_armor_center = Point(0, 0);
            Return_zero();//归零
#if CALL_KALMAN == 1
            kalman.reset();//卡尔曼清零
#endif
        }
        //释放内存
        img.free_Memory();
//串口传输
#if CALL_SERIALPORT == 1
        serial.RMserialWrite(_yaw, fabs(yaw)*100, _pitch, fabs(pitch)*100, depth, data_type, is_shooting);
#endif 
            break;

        //大神符
        default:
      
            break;
        }
    
    imshow("frame", frame);
    //清空相机内存
    cap.cameraReleasebuff();
    // /* "Esc"-退出 */
    // if (waitKey(1) == 'q') 
    // {
    //   break;
    // }
        //输出帧率
#if FPS_SHOW == 1
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency(); //结束计时
    int fps = int(1.0 / t);                                        //转换为帧率
    cout << "FPS: " << fps << endl;                                //输出帧率
#endif
  }
}
void WorKing::Return_zero()
{
    is_shooting = 0;
    data_type = 0;
    img.armor_success = false;
    img.armor_roi = Rect(0, 0, 0, 0);
    // img.roi_num = 0;
    img.switch_armor = false;
    img.lost_armor_success = img.armor_success;
    yaw = 0;
    pitch = 0;
    _yaw = 0;
    _pitch = 0;     
}

/**
 * @brief 模式选择
 *
 */
void WorKing::Mode_Selection()
{
    int ctrl_arr[REC_BUFF_LENGTH];
    serial.RMreceiveData(ctrl_arr);
    enemy_color = ctrl_arr[1];
    pattern = ctrl_arr[2];
    firing = ctrl_arr[3];
}
/**
 * @brief 角度补偿
 * 
 */
void WorKing::Angle_compensate()
{

    if (_offset_x == 0)
    {
        yaw = yaw - offset_x / 100;
    }
    else
    {
        yaw = yaw + offset_x / 100;
    }
    int dist = depth;
    // cout<<dist<<endl;
    // if(firing == 1)
    // {
    //     offset_y = 0.1616 * dist + 96.644;
    // }
    // else if(firing == 2)
    // {
    //     offset_y = 0.0517+127.69;
    // }
    // else if(firing == 3)
    // {
    //     if(dist < 2000)
    //     {
    //     offset_y = 100;
    //     }
    //     else if(dist>=2000)
    //     {
    //         switch (dist/1000)
    //         {
    //         case 3:
    //         offset_y = 300;
    //         break;
    //         case 4:
    //         offset_y = 300;
    //         break;
    //         case 5:
    //         offset_y = 400;
    //         break;
    //         case 6:
    //         offset_y = 400;
    //         break;
    //         default:
    //         break;
    //         }
    //     }
    // }
    // pitch = pitch - offset_y / 100;
    if (yaw > 0)
    {
        _yaw = 0;
    }
    else
    {
        _yaw = 1;
    }
    if (pitch > 0)
    {
        _pitch = 0;
    }
    else
    {
        _pitch = 1;
    }
}
/**
 * @brief 自动开火
 * 
 */
void WorKing::Automatic_fire()
{
    line(frame, Point(0 , CAMERA_RESOLUTION_ROWS/2), Point(CAMERA_RESOLUTION_COLS, CAMERA_RESOLUTION_ROWS/2), Scalar(0, 255, 255));
    line(frame, Point(CAMERA_RESOLUTION_COLS/2 , 0), Point(CAMERA_RESOLUTION_COLS/2, CAMERA_RESOLUTION_ROWS), Scalar(0, 255, 255));
    
    // cout<<"depth = "<<depth<<endl;
    float variance = 0;//方差小于一定数值为周期性变化
    if(img.roi_num_law[4] > 0)
    {
        float sum = 0;
        for(int i = 0; i < 5; i++)
        {
            sum+=img.roi_num_law[i];
        }
        sum = sum/5;
        for(int i = 0; i< 5; i++)
        {
            variance += pow(img.roi_num_law[i] - sum ,2);
        }
        variance = sqrt(variance/5);
    }
    cout<<"yaw = "<<yaw<<endl;
    //装甲板范围
    if(img.armor[img.optimal_armor].distinguish > 0)
    {
        offset_yaw = 6.5441*exp(-0.001*depth) + 2;
    }
    else{
        offset_yaw = 6.5441*exp(-0.001*depth) + 0.5;
    }
    
    cout<<"offset_yaw = "<<offset_yaw<<endl;
    //自动开火判断
    if(fabs(yaw) <= offset_yaw && fabs(img.armor[img.optimal_armor].tan_angle) < 5 && depth < 5000) 
    {
        if(variance == 0)
        {
            //一直ROI且没有周期性变化

            if(img.roi_num > 10 && fire_num > 5)
            {
                is_shooting = 2;//自动
                cout<<"piu piu piu piu piu"<<endl;
            }
            else if(img.roi_num > 2)
            {   
                is_shooting = 1;//单发
                cout<<"piu"<<endl;
            }
        }
        else{
            //有周期性变化
            
            if(variance > 2 && img.roi_num > 10)
            {
                is_shooting = 2;//自动
                cout<<"piu piu piu piu piu"<<endl;
            }
            else if(variance < 2 && img.roi_num > 2)
            {
                // cout<<"小陀螺"<<endl;
                is_shooting = 1;//单发
                cout<<"piu"<<endl;
            }
            
        }
        //开火计数
        fire_num++;
    }
}


