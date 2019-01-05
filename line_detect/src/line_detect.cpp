#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<Eigen/Core>
#include<Eigen/Geometry>
#include <iostream>

static const std::string OPENCV_WINDOW = "Image window";
static const std::string GRAY = "Gray";
static const std::string BLUR = "HSV";
static const std::string CANNY = "Canny";
using namespace cv;
using namespace std;
using namespace Eigen;

void extract_black(Mat* src, Mat* dst, double th_g, int skip_step_);
void calc_variance(Mat* src, vector<int>* variance, int th, int direction, int skip_step_);
void get_filtered(Mat* src, Mat* filtered);
Vector2d transform_point_homography(Vector2d point);

class ImageConverter{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_){
      image_sub_ = it_.subscribe("/bebop/image_raw", 1, &ImageConverter::imageCb, this);
      //ESTO LO HAGO PARA TRABAJAR SIN TENER QUE TENER EL BEBOP PRENDIDO TODO EL TIEMPO
      //image_sub_ = it_.subscribe("/camera/image", 1, &ImageConverter::imageCb, this);
      image_pub_ = it_.advertise("/image_converter/output_video", 1);

      cv::namedWindow(OPENCV_WINDOW);
      cv::namedWindow(GRAY);
      cv::namedWindow(BLUR);
      cv::namedWindow(CANNY);
      //cv::namedWindow(PLACEHOLDER);
    }

    ~ImageConverter(){
      cv::destroyWindow(OPENCV_WINDOW);
      cv::destroyWindow(GRAY);
      cv::destroyWindow(BLUR);
      cv::destroyWindow(CANNY);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg){
      cv_bridge::CvImagePtr cv_ptr;
      cv_bridge::CvImagePtr cv_ptr_copy;
      try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv_ptr_copy = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
      //PRUEBA HSV
      Mat hsvImg;
      vector<Mat> hsvChannels;
      cvtColor(cv_ptr->image, hsvImg, CV_BGR2HSV);

      split(hsvImg, hsvChannels);
      //HOUGH
      
      //value mask
      Mat valueChannel = hsvChannels[2];
      
      Mat hueMask, saturationMask, valueMask;

      inRange(hsvImg, Scalar(0, 0, 0), Scalar(179, 50, 100), valueMask);

      if(cv_ptr->image.empty())
      {
         cout << "ERROR" << endl;
      }
      //tamaño imagen: height = 480; width = 856
      //Recortamos la imagen a lo largo por conveniencia y facilidad de procesamiento
      cv_ptr_copy->image = cv_ptr_copy->image(Rect(0, 400, 856, 40));
      //valueMask = valueMask(Rect(0, 400, 856, 40));

      //EN PRINCIPIO, ES NECESARIO APLICAR FILTROS PARA DETECTAR UNA LINEA NEGRA
      //Se extrae la parte negra de la imagen
      Mat gray, blurr, edge, tresh, sobel, grad_x, grad_y, abs_grad_x, abs_grad_y;
      vector<Vec4i> lines;

      cvtColor(cv_ptr_copy->image, gray, CV_BGR2GRAY);

      GaussianBlur(gray, blurr, Size(3, 3), 0, 0);
      //GaussianBlur(valueMask, blurr, Size(3, 3), 0, 0);
      Canny(blurr, edge, 120, 180, 3);
      HoughLinesP(edge, lines, 1, CV_PI/180, 5, 2, 1);

      for(size_t i=0; i<lines.size(); i++){
        Vec4i l = lines[i];
        line(cv_ptr->image(Rect(0, 400, 856, 40)), Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 255, 0), 2, 0);
      }
      //cv::imshow(OPENCV_WINDOW, cv_ptr_copy->image);
      cv::imshow(OPENCV_WINDOW, cv_ptr->image);
      
      //Debugs visuales
      cv::imshow(BLUR, valueMask);
      cv::imshow(GRAY, gray);
      cv::imshow(CANNY, edge);
      cv::waitKey(3);

      image_pub_.publish(cv_ptr_copy->toImageMsg());
    }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}