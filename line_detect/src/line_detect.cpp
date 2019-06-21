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

//USED FOR DEBUGS
static const std::string OPENCV_ORIGINAL = "Original Image";
static const std::string OPENCV_CROP = "Crop";
static const std::string OPENCV_GRAY = "Gray";
static const std::string OPENCV_BLUR = "Blur";
static const std::string OPENCV_THRESH = "Thresh";
static const std::string OPENCV_CONTOURS = "Contours";

static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_BIN = "Binary Image";


using namespace cv;
using namespace std;
using namespace Eigen;

geometry_msgs::Twist msg;
geometry_msgs::Twist camera_msg;
cv_bridge::CvImagePtr cv_ptr;

ros::Publisher line_pos;
ros::Publisher vcamera_pub;
image_transport::Publisher camera_pub;

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
      image_transport::ImageTransport it(n);
      line_pos=n.advertise<geometry_msgs::Point>("Line_position", 1000);
      camera_pub = it.advertise("Processed_image", 1);
    }
    ~ImageProcess(){
      cv::destroyAllWindows();
    }

    void detect_line(cv::Mat& img){
      // ANTES DE EMPEZAR, MOVEMOS LA CÁMARA VIRTUAL 70 GRADOS EN PITCH, PARA ASEGURAR QUE EL DRONE VE EL PISO
      camera_msg.angular.y = -40;
      vcamera_pub.publish(camera_msg);
      // DECLARACIÓN DE VARIABLES NECESARIAS
      Mat processed_img, hsvImg, gray, blurr, edge, tresh, sobel, grad_x, grad_y, abs_grad_x, abs_grad_y, valueMask;
      
      vector<vector<Point> > contours;
      vector<Vec4i> lines, hierarchy;
      Point2f center;
      //MENSAJE DE GEOMETRÍA CON LAS COORDENADAS DEL PUNTO DEL CENTROIDE (X, Y)
      geometry_msgs::Point mid_pos;
      //VARIABLES NECESARIAS PARA CALCULAR EL CENTROIDE POR CON BASE EN LAS LINEAS 
      float radius;
      float x1=img.cols/2, y1=img.rows+1, x2=img.cols/2, y2=-1.0;
      
      //SE INICIALIZAN LAS COORDENADAS DEL CENTROIDE EN 0, ESTO PARA ASEGURAR QUE SE PUBLICA UN MENSAJE CON
      //(0,0) SI EL PUNTO NO EXISTE O (X, Y) SI EL PUNTO EXISTE
      mid_pos.x= 0;
      mid_pos.y = 0;

      //cv::imshow(OPENCV_ORIGINAL, img); //MOSTRAMOS LA IMAGEN ORIGINAL PARA REFERENCIA
      
      // == PRIMER PASO == PROCESAMIENTO DE LA IMAGEN
      //Recortamos la imagen para conseguir la región de interes
      img = img(Rect(308, 400, 200, 80));
      cv::imshow(OPENCV_CROP, img); //MOSTRAMOS LA IMAGEN CON LA REGIÓN DE INTERES PARA REFERENCIA
      /*
      * PRESENTAMOS DOS FORMAS DE PROCESAR LA IMAGEN: 
      * 1- POR MEDIO DE UMBRALIZACIÓN DE ESCALAS DE GRISES (MÁS EFICIENTE PARA ESTE TRABAJO)
      * 2- POR MEDIO DE DISCRIMINACIÓN DE COLORES (MENOS EFICIENTE PARA ESTE TRABAJO, PERO EXTENDIBLE
      *     PARA FUTUROS TRABAJOS, EN LOS CUALES SE IRÁ A REQUERIR DISCRIMINAR POR COLORES)
      */

      //1 - POR MEDIO DE UMBRALIZACIÓN (ESTE METODO SE UTILIZARÁ PARA ESTE TRABAJO)
      //cvtColor(img, hsvImg, CV_BGR2GRAY); //CONVERTIMOS LA IMAGEN DE ENTRADA EN ESCALA DE GRISES
      //cv::imshow(OPENCV_GRAY, hsvImg); //PARA REFERENCIA MOSTRAMOS LA IMAGEN CONVERTIDA A ESCALA DE GRISES

      //GaussianBlur(hsvImg, blurr, Size(3, 3), 0, 0); //APLICAMOS FILTRO GAUSSIANO PARA ELIMINAR RUIDO
      //cv::imshow(OPENCV_BLUR, blurr); //MOSTRAMOS LA IMAGEN PROCESADA CON EL FILTRO PARA REFERENCIA

      //APLICAMOS EL UMBRAL, PARA NEGRO EL RANGO IDEAL ES ~60, THRESH_BINARY_INV INDICA QUE VALORES TOMARÁ
      //LA NUEVA IMAGEN: 0 SI src(x, y)>thresh; 255 de lo contrario
      //threshold(hsvImg, processed_img, 100, 255, THRESH_BINARY_INV); 
      //cv::imshow(OPENCV_THRESH, processed_img); //MOSTRAMOS LA IMAGEN UMBRALIZADA CON PARA REFERENCIA

      //2 - POR MEDIO DE DISCRIMINACIÓN DE COLORES
      cvtColor(img, hsvImg, CV_BGR2HSV); //CONVERTIMOS LA IMAGEN DE ENTRADA EN HSV
      //hsvImg = ~hsvImg;
      inRange(hsvImg, Scalar(0, 0, 128), Scalar(255, 255, 255), valueMask); //OBTENEMOS LA MÁSCARA SEGÚN EL RANGO DE VALORES
      bitwise_and(img, img, processed_img, valueMask); //APLICAMOS LA MÁSCARA A LA IMAGEN ORIGINAL, ELIMINANDO LOS PIXELES QUE NO INTERESAN
      //GaussianBlur(processed_img, blurr, Size(3, 3), 0, 0); //APLICAMOS FILTRO GAUSSIANO
      //Canny(blurr, edge, 120, 180, 3); //RESALTAMOS BORDES CON CANNY
      threshold(processed_img, processed_img, 60, 255, THRESH_BINARY_INV);
      cvtColor(processed_img, processed_img, CV_BGR2GRAY);
      cv::imshow(OPENCV_THRESH, processed_img);
      //OBTENEMOS LOS CONTORNOS DE LA IMAGEN
      //findContours(processed_img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
      
      //OBTENEMOS EL CENTROIDE POR MEIDO DE LOS MOMENTOS DE HU
      /*
      vector<Moments> mu(contours.size());
      vector<Point2f> mc(contours.size());
      Mat draw = Mat::zeros(processed_img.size(), CV_8UC3);
      for(int i = 0; i<contours.size(); i++){
        mu[i]=moments(contours[i], false);
      }
      */
      Moments mu;
      Point2f mc;
      //Mat draw = Mat::zeros(processed_img.size(), CV_8UC3);
      mu=moments(processed_img, false);
      
      mc=Point2f((mu.m10/(mu.m00)), (mu.m01/(mu.m00)));
      
      mid_pos.x = mc.x;
      mid_pos.y = mc.y;

      circle(processed_img, mc, 4, Scalar(0, 0, 0), -1);
      //if(mc.size()>0){
        //std::cout << mc[0] << "\n";
        //mid_pos.x = mc[0].x;
       // mid_pos.y = mc[0].y;
      //}
      //for(int i =0; i<contours.size(); i++){
      //  Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        //drawContours(draw, contours, i, color, 2);
      //  circle(processed_img, mc[i], 4, color, -1);
      //}
      
      //DRAW CONTOURS:
      /*
      *
      * ESTE SEGMENTO ES UTILIZADO PARA CALCULAR EL CENTROIDE CON BASE EN LAS LINEAS, TOMANDO EL PUNTO DE Y MÁS
      * ALTO Y EL PUNTO DE Y MÁS BAJO, Y SUS VALORES DE X ASOCIADOS, CALCULAMOS EL PUNTO INTERMEDIO
      * 
      * /
      /*Mat draw = Mat::zeros(edge.size(), CV_8UC3);
      if(contours.size() > 0){
        for(int i=0; i<contours.size(); i++){CV_GRAY2BGR
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
        mid_pos.x = (x1+x2)/2;
        mid_pos.y = (y1+y2)/2;
        circle(processed_img, Point((center.x+x2)/2, (center.y+y2)/2), 5, (0, 255, 200), 2);
        circle(img, Point(x2, y2), 5, (0, 0, 255), 1);
      }*/

      //PUBLICAMOS LOS MENSAJES RELEVANTES (POSICIÓN DE CENTROIDE, IMAGEN PROCESADA) Y MOSTRAMOS LA IMAGEN PROCESADA
      line_pos.publish(mid_pos);
      cv_bridge::CvImage img_bridge;
      sensor_msgs::Image img_msg;
      std_msgs::Header header;
      //cvtColor(processed_img, processed_img, CV_GRAY2BGR);
      img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img);
      img_bridge.toImageMsg(img_msg);
      camera_pub.publish(img_msg);
      //cv::imshow(OPENCV_WINDOW, draw);
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
  vcamera_pub = nh_.advertise<geometry_msgs::Twist>("/bebop/camera_control", 1000);
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
