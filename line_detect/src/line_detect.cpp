#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <stdlib.h>

static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_BIN = "Binary Image";

using namespace cv;
using namespace std;
using namespace Eigen;

geometry_msgs::Twist msg;
cv_bridge::CvImagePtr cv_ptr;

ros::Publisher line_pos;

RNG rng(12345);
int line_exists_flag;
int countdown_end = 5;
bool end_flag = false;
void extract_black(Mat* src, Mat* dst, double th_g, int skip_step_);
void calc_variance(Mat* src, vector<int>* variance, int th, int direction, int skip_step_);
void get_filtered(Mat* src, Mat* filtered);
Vector2d transform_point_homography(Vector2d point);

class ImageProcess{

public:
    ImageProcess(){
      ros::NodeHandle n;
      line_pos=n.advertise<geometry_msgs::Point>("Line_position", 1000);
    }
    ~ImageProcess(){
      cv::destroyAllWindows();
    }

    void detect_line(cv::Mat& img){
      Mat processed_img;
      vector<Mat> hsvChannels;
      vector<vector<Point> > contours;
      Mat hsvImg, gray, blurr, edge, tresh, sobel, grad_x, grad_y, abs_grad_x, abs_grad_y, valueMask;
      vector<Vec4i> lines, hierarchy;
      Point2f center;
      geometry_msgs::Point mid_pos;
      float radius;
      float x1=img.cols/2, y1=img.rows+1, x2=img.cols/2, y2=-1.0;
      mid_pos.x= 0;
      mid_pos.y = 0;
      //Recortamos la imagen para conseguir la región de interes
      img = img(Rect(308, 400, 200, 80));
      //HSV
      cvtColor(img, hsvImg, CV_BGR2HSV);
      //img = 255-img;
      //cvtColor(img, hsvImg, CV_BGR2GRAY);
      //hsvImg = 255-hsvImg;
      //threshold(hsvImg, processed_img, 180, 255, THRESH_BINARY_INV);

      //split(hsvImg, hsvChannels);
      //Para un rango de negro* (Este valor lo calculé para la imagen que tengo, puede no servir, pero se puede ajustar)
      inRange(hsvImg, Scalar(0, 0, 0), Scalar(180, 255, 255), valueMask);
      //bitwise_img
      
      bitwise_and(img, img, processed_img, valueMask);
      //Convirtiendo la imagen a escala de grises
      //cvtColor(processed_img, gray, CV_BGR2GRAY);
      //Aplicando filtro gaussiano
      GaussianBlur(processed_img, blurr, Size(3, 3), 0, 0);
      //GaussianBlur(valueMask, blurr, Size(3, 3), 0, 0);
      //Detectando bordes con Canny
      Canny(blurr, edge, 120, 180, 3);
      //CONTOURS
      findContours(edge, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

      //HOUGH
      //HoughLinesP(edge, lines, 1, CV_PI/180, 1, 3, 16);
      //value mask
      //Mat valueChannel = hsvChannels[2];
      //Mat hueMask, saturationMask, valueMask;
      
      //DRAW CONTOURS:
      Mat draw = Mat::zeros(edge.size(), CV_8UC3);
      if(contours.size() > 0){
        for(int i=0; i<contours.size(); i++){
          minEnclosingCircle(contours[i], center, radius);
          if(center.y<y1){
            y1=center.y;
            x1=center.x;
          }
          if(center.y>y2){
            y2=center.y;
            x2=center.x;
          }
          Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
          drawContours(draw, contours, i, color, 2, 8, hierarchy, 0, Point());
        }
        mid_pos.x = (center.x+x2)/2;
        mid_pos.y = (center.y+y2)/2;
        //std::cout<<"DETECTED CENTER:";
        //std::cout<<center<<std::endl;
        //std::cout<<"CALCULATED ONE:";
        //std::cout<<Point(x1, y1)<<std::endl;
        //std::cout<<"CALCULATED TWO:";
        //std::cout<<Point(x2, y2)<<std::endl;
        //std::cout<< "WIDTH : " << processed_img.cols << " HEIGHT: " << processed_img.rows << "\n";
        circle(processed_img, Point((center.x+x2)/2, (center.y+y2)/2), 5, (0, 255, 200), 2);
        circle(img, Point(x2, y2), 5, (0, 0, 255), 1);
      }
      line_pos.publish(mid_pos);
      //std::cout << processed_img.cols << std::endl;
      cv::imshow(OPENCV_WINDOW, draw);
      cv::imshow(OPENCV_BIN, processed_img);
    }


    void imageCallback(const sensor_msgs::ImageConstPtr& msg){
      try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat img_copy = cv_ptr->image.clone();
        detect_line(img_copy);
        int get_input = cv::waitKey(30);
        if(get_input==27){
          end_flag = true;
        }
      }catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
    }
};


int main(int argc, char** argv){
  ros::init(argc, argv, "image_converter");
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_(nh_);
  image_transport::Subscriber image_sub_;
  ImageProcess ip;
  image_sub_ = it_.subscribe("/bebop/image_raw", 1, &ImageProcess::imageCallback, &ip);
  //ESTO LO HAGO PARA TRABAJAR SIN TENER QUE TENER EL BEBOP PRENDIDO TODO EL TIEMPO
  //image_sub_ = it_.subscribe("/camera/image", 1, &ImageProcess::imageCallback, &ip);
  
  while(nh_.ok()){
    ros::spinOnce();
    if(end_flag){
      break;
    }
  }
  cv::destroyAllWindows();
  return 0;
}
