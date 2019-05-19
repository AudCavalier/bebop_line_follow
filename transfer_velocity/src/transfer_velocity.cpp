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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <stdlib.h>

using namespace cv;
using namespace std;
using namespace Eigen;

static const std::string FAKE_WINDOW = "Fake window";

std_msgs::Empty takeoff_msg, land_msg;
geometry_msgs::Twist vel_bebop;

ros::Publisher takeoff_pub; //tópico para despegar
ros::Publisher land_pub; //tópico para aterrizar
ros::Publisher vel_pub; //tópico para mover el bebop
ros::Subscriber sub; //para subscribirse al tópico del punto 

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

    void velHandle(const geometry_msgs::Point &msg){
      //Si Point.y >0 MOVER, si no DETENER
      //Si Point.x >width/2+20 || Point.x < width/2-20 -> girar, si no, no girar
      //std::cout << "OK\n";
      int y_point = msg.y;
      int x_point = msg.x;
      float pitch = 0.0, yaw = 0.0;
      //std::cout << msg << std::endl;
      if(can_move){ 
        if (y_point>0){
          //std::cout << "MOVING FORWARD\n";
          pitch = 0.05;
        }else{
          pitch = 0.0;
        }
        vel_bebop.linear.x = pitch;
        //Recordatorio personal. angular.z >0 contrario a las manecillas
        //angular.z <0 en sentido de las manecillas del reloj
        if (x_point<25){
          //std::cout << "ROTATING\n";
          //detenemos primero el bebop para que gire
          vel_bebop.linear.x = 0;
          //vel_bebop.linear.y = 0;
          //vel_bebop.linear.z = 0;
          vel_bebop.angular.z = 0.3;
        }else if(x_point>55){
          vel_bebop.linear.x = 0;
          //vel_bebop.linear.y = 0;
          //vel_bebop.linear.z = 0;
          vel_bebop.angular.z = -0.3;
        }else{
          vel_bebop.angular.z = 0.0;
        }
        //std::cout << "inb4" << "\n";
        std::cout << "NOT ROTATING: X VEL: " << vel_bebop.linear.x << "\n Y VEL: " <<  vel_bebop.linear.y;
        std::cout << " Z VEL: " << vel_bebop.linear.z << " ANGULAR: ";
        std::cout << vel_bebop.angular.z << "\n";
        //vel_bebop.angular.z = 0;
        //std::cout << "Pitch: " << pitch << " Yaw: " << yaw << "\n";
        vel_pub.publish(vel_bebop);
      }
      else{
        vel_bebop.linear.x = 0;
        vel_bebop.linear.y = 0;
        vel_bebop.linear.z = 0;
        vel_bebop.angular.z = 0;
        vel_pub.publish(vel_bebop);
     }
    }

    void chatterCallback(const geometry_msgs::Point &msg){
      velHandle(msg);
    }

    void fakeimgCallback(const sensor_msgs::ImageConstPtr &msg){
      //Este método es simplemente para poder acceder al teclado con cv::waitKey
      try{
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
  //fake_image_sub_ = it_.subscribe("/camera/image", 1, &VelocityHandle::fakeimgCallback, &vh);
  fake_image_sub_ = it_.subscribe("/bebop/image_raw", 1, &VelocityHandle::fakeimgCallback, &vh);
  //ESTO LO HAGO PARA TRABAJAR SIN TENER QUE TENER EL BEBOP PRENDIDO TODO EL TIEMPO
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