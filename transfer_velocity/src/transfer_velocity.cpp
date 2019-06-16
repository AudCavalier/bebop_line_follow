#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Empty.h>
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

using namespace cv;
using namespace std;
using namespace Eigen;

static const std::string FAKE_WINDOW = "Fake window";

cv_bridge::CvImagePtr cv_ptr;

std_msgs::Empty takeoff_msg, land_msg;
geometry_msgs::Twist vel_bebop;

ros::Publisher takeoff_pub; //tópico para despegar
ros::Publisher land_pub; //tópico para aterrizar
ros::Publisher vel_pub; //tópico para mover el bebop
ros::Subscriber sub; //para subscribirse al tópico del punto
ros::Subscriber img;

Point2f bebop_center;
bool end_flag = false;
int can_move = 0; //variable para indicar que el bebop empezará a moverse(1) o no (0)

class VelocityHandle{  
public:
  VelocityHandle(){
      ros::NodeHandle n;
      sub = n.subscribe("Line_position", 1000, &VelocityHandle::chatterCallback, this);
      vel_bebop.linear.x = 0;
      vel_bebop.linear.y = 0;
      vel_bebop.linear.z = 0;
    }

    ~VelocityHandle(){
      //De-constructor
    }

    void calccenter(cv::Mat& img){
      Point2f img_center;
      img_center.x = img.cols/2;
      img_center.y = img.rows/2;
      bebop_center = img_center;
    }

    void hover(){
      vel_bebop.linear.x = 0;
      vel_bebop.linear.y = 0;
      vel_bebop.linear.z = 0;
      vel_bebop.angular.z = 0;
      vel_pub.publish(vel_bebop);
    }

    void velHandle(const geometry_msgs::Point &msg){
      int y_point = msg.y;
      int x_point = msg.x;
      float pitch = 0.0, yaw = 0.0;
      std::cout << "DEBUG MSG : ==================== " << msg << std::endl;
      std::cout << "DEBUG: X POINT: " << x_point << " Y POINT: " << y_point << "\n";
      if(can_move){
        if(x_point>=0 &&  y_point>=0){
          //Recordatorio personal. angular.z >0 contrario a las manecillas
          //angular.z <0 en sentido de las manecillas del reloj
          if(x_point < bebop_center.x-75){
            if(y_point>bebop_center.y-5 && y_point < bebop_center.y+5){
              std::cout << "DEBUG : : MOVING LEFT : :" << "\n";//x_point << " :: " << bebop_center << "\n";
              vel_bebop.linear.x = 0;
              vel_bebop.linear.y = 0.02;
              vel_bebop.linear.z = 0;
              vel_bebop.angular.z = 0.0;
            }else{
              std::cout << "DEBUG : : TURNING LEFT : :" << x_point << " :: " << bebop_center << "\n";
              vel_bebop.linear.x = 0;
              vel_bebop.linear.y = 0;
              vel_bebop.linear.z = 0;
              vel_bebop.angular.z = 0.1;
            }
          }
          else if(x_point > bebop_center.x+75){
             if(y_point>bebop_center.y-5 && y_point < bebop_center.y+5){
              std::cout << "DEBUG : : MOVING RIGHT : :" << "\n";//x_point << " :: " << bebop_center << "\n";
              vel_bebop.linear.x = 0;
              vel_bebop.linear.y = -0.02;
              vel_bebop.linear.z = 0;
              vel_bebop.angular.z = 0.0;
            }else{
              std::cout << "DEBUG : : TURNING RIGHT : :" << x_point << " :: " << bebop_center << "\n";
              vel_bebop.linear.x = 0;
              vel_bebop.linear.y = 0;
              vel_bebop.linear.z = 0;
              vel_bebop.angular.z = -0.1;
            }
          }
          else if(y_point > bebop_center.y-(bebop_center.y/4)){
            std::cout << "MOVING FORWARD";
            std::cout << " X VEL: " << vel_bebop.linear.x << "\n Y VEL: " <<  vel_bebop.linear.y;
            std::cout << " Z VEL: " << vel_bebop.linear.z << " ANGULAR: ";
            std::cout << vel_bebop.angular.z << "\n";
            vel_bebop.linear.y = 0.0;
            vel_bebop.angular.z = 0.0;  
            pitch = 0.018;
          }else{
            std::cout << "STANDING STILL\n";
            vel_bebop.linear.x = 0;
            vel_bebop.linear.y = 0;
            vel_bebop.linear.z = 0;
            vel_bebop.angular.z = 0;
          }
          //vel_bebop.angular.z = 0;
          //std::cout << "Pitch: " << pitch << " Yaw: " << yaw << "\n";
          vel_bebop.linear.x = pitch;
          vel_pub.publish(vel_bebop);
        }
      }
      else{
        hover();
     }
    }

    void chatterCallback(const geometry_msgs::Point &msg){
      velHandle(msg);
    }

    void fakeimgCallback(const sensor_msgs::ImageConstPtr &msg){
      //Este método es simplemente para poder acceder al teclado con cv::waitKey
      try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat img_copy = cv_ptr->image.clone();
        calccenter(img_copy);
        cv::Mat1f Fake_img(1, 1, 0.0f);
        cv::imshow(FAKE_WINDOW, Fake_img);
        int get_input = cv::waitKey(30);
        if(get_input==27){
          end_flag = true;
        }
        if(get_input == 't'){
          //std::cout << "KEY: " << get_input << " TAKE OFF\n";
          ROS_INFO("DESPEGANDO...");
          //DESCOMENTAR PARA USAR EL NODO :: TENER PRECAUCIÓN
          takeoff_pub.publish(takeoff_msg);
        }
        if(get_input == 'l'){
          //std::cout << "KEY: " << get_input << " LAND\n";
          ROS_INFO("ATERRIZANDO...");
          land_pub.publish(land_msg);
        }
        if(get_input == 's'){
          //std::cout << "KEY: " << get_input << " LAND\n";
          can_move = 1 - can_move;
          if(can_move){
            ROS_INFO("INICIANDO...");
          }else{
            ROS_INFO("TERMINANDO...");
          }
        }
      }catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
    }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "velocity_handle");
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_(nh_);
  image_transport::Subscriber fake_image_sub_;
  VelocityHandle vh;
  //ESTO LO HAGO PARA TRABAJAR SIN TENER QUE TENER EL BEBOP PRENDIDO TODO EL TIEMPO
  //fake_image_sub_ = it_.subscribe("/camera/image", 1, &VelocityHandle::fakeimgCallback, &vh);
  fake_image_sub_ = it_.subscribe("Processed_image", 1, &VelocityHandle::fakeimgCallback, &vh);
  std::cout << "Menú:\nPresionar 't' para despegar\nPresionar 'l' para aterrizar\n";
  std::cout << "Presionar 's' para iniciar/terminar (el drone debe estár en el aire)\nPresionar 'esc' para salir\n";
  takeoff_pub = nh_.advertise<std_msgs::Empty>("/bebop/takeoff", 1000);
  land_pub = nh_.advertise<std_msgs::Empty>("/bebop/land", 1000);
  vel_pub = nh_.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 1000);
  while(nh_.ok()){
    ros::spinOnce();
    if(end_flag){
      break;
    }
  }
  cv::destroyWindow(FAKE_WINDOW);
  return 0;
}