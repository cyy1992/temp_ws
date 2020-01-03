#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
RNG g_rng(12345);//毛大大的博客里看到的生成随机数，用于生成随机颜色
bool is_rect = false;//不可避免地还是要定义几个全局变量，伤心
Point start_position=Point(-1,-1);
Point end_position= Point(-1, -1);
Mat last_img;
Mat bool_img,last_bool_img;
vector<int> rect_manu;
void on_mouse(int event, int x, int y, int flags, void *ustc)
//event鼠标事件代号，x,y鼠标坐标，flags拖拽和键盘操作的代号    
{
  Mat& image = *(cv::Mat*) ustc;//这样就可以传递Mat信息了，很机智
  char temp[16];
  switch (event) {
    case CV_EVENT_LBUTTONDOWN://按下左键
    {
      last_img = image.clone();
      last_bool_img = bool_img.clone();
//       sprintf(temp, "(%d,%d)", x, y);
//       putText(image, temp, Point(x, y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0, 255));
      is_rect = true;
      start_position= Point(x, y);
    } break;
    case CV_EVENT_MOUSEMOVE://移动鼠标
    {
      end_position = Point(x, y);
      if (is_rect)
      { }
    }break;
    case CV_EVENT_LBUTTONUP:
    {
      is_rect = false;
//       sprintf(temp, "(%d,%d)", x, y);
//       putText(image, temp, Point(x, y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0, 255));
      //调用函数进行绘制
      cv::rectangle(image,start_position, end_position, cv::Scalar(g_rng.uniform(0, 255), g_rng.uniform(0, 255), g_rng.uniform(0, 255)));
      for(int i = start_position.x; i < end_position.x; i++)
      {
        for(int j = start_position.y; j < end_position.y; j++)
        {
          bool_img.at<uchar>(j,i) = 0;
        }
      }
      
    }break;
    
    case CV_EVENT_MBUTTONDOWN:
    {
      image = last_img;
      bool_img = last_bool_img;
    }break;
  }
}

void drawBlack(vector<int> rect_manu)
{
  for(auto a : rect_manu)
  {
    cout << a <<endl;
  }
  
  for(int i = rect_manu[0]; i < rect_manu[2]; i++)
  {
    for(int j = rect_manu[1]; j < rect_manu[3]; j++)
    {
      bool_img.at<uchar>(j,i) = 0;
    }
  }
}
int main(int argc, char **argv) {
    std::cout << "Hello, world!" << std::endl;
    string map_name = argv[1];
    double display_ratio = atof(argv[2]);
    string img_path = std::string(getenv("HOME")) + "/map/" +  map_name +"/";
    
    Mat img = imread(img_path + "map.png");
//     cout << (img.type() == CV_8UC3 )<<endl;
    Mat img_resize,result;
    Size origin_size = Size(img.cols, img.rows);
    cout << "origin_size: " << img.cols <<", " << img.rows <<endl;
//     threshold(img, result, 30, 200.0, CV_THRESH_BINARY);
    Size ResImgSiz = Size(img.cols*display_ratio, img.rows*display_ratio);
    resize(img, img_resize,ResImgSiz);
    last_img = img_resize;
    Mat temp1;
    int key_value;
    Mat origin_bool_image = imread(img_path + "bool_image.png",CV_8UC1);
    if(origin_bool_image.empty())
    {
      bool_img =  Mat(ResImgSiz,CV_8UC1,Scalar(255)); 
    }
    else{
      cout <<( origin_bool_image.type() == CV_8UC1) <<endl;
      resize(origin_bool_image, bool_img, ResImgSiz);
    }
//     bool_img =  Mat(ResImgSiz,CV_8UC1,Scalar(255)); 
    last_bool_img = bool_img.clone();
    while ( key_value != 's' && key_value != 'S') {
      img_resize.copyTo(temp1);
      namedWindow("img");
      setMouseCallback("img", on_mouse, (void*)&img_resize);  
      if(is_rect)
        rectangle(temp1, start_position, end_position, cv::Scalar(g_rng.uniform(0, 255), g_rng.uniform(0, 255), g_rng.uniform(0, 255)));
//       putText(temp1,"("+std::to_string(end_position.x)+","+std::to_string(end_position.y)+")" , end_position, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0, 255));
      imshow("img",temp1);
      imshow("bool_img", bool_img);
      key_value = waitKey(30);
      if(key_value == 32)
      {
        rect_manu.clear();
        cout << "please input rect num: " <<endl;
        int b;
        while(cin>>b)
        {
          rect_manu.push_back(b*display_ratio);
          cout << b <<endl;
          if (cin.get() == '\n') 
            break;
        }
        drawBlack(rect_manu);
      }
    }
    
    resize(bool_img, result,origin_size);
    imwrite(img_path + "bool_image.png",result);
    cout << "result size: " << result.cols <<", " << result.rows <<endl;
    cout << "write "+ img_path+"bool_image.png done! " <<endl;
//     imshow("1",img_resize);
//     waitKey(0);
    return 0;
}
