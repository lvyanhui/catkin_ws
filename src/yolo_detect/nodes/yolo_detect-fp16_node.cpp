#include "yolo_detect/yolo_detect-fp16.h" 

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "yolo_detect/Rect.h"
#include "yolo_detect/Object.h"
#include "yolo_detect/ObjectVec.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";
static std::string get_object_classic(int class_id);
static Detector* pDetector;
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher chatter_pub;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    //image_sub_ = it_.subscribe("/camera/image_raw", 1,
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    chatter_pub = nh_.advertise<yolo_detect::ObjectVec>("chatter", 1000);
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    ROS_INFO("cv_bridge info, %d", pDetector->model_file);
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    {
      //cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
      
      std::vector<cv::Rect > rects;
      std::vector<int > class_id;
      std::vector<float > confidences;
      rects.clear();
      class_id.clear();
      confidences.clear();
      ROS_INFO("rect nums" );
      pDetector->Detect(cv_ptr->image, rects, class_id, confidences);
      ROS_INFO("rect nums-%lu", rects.size());
      
      yolo_detect::ObjectVec msg;
      yolo_detect::Object obj;
    
      for(int i=0; i<rects.size(); i++)
      {
        obj.classid = class_id[i];
        obj.confidence = confidences[i];
        obj.rect.top = rects[i].x;
        obj.rect.left = rects[i].y;
        obj.rect.width = rects[i].width;
        obj.rect.height = rects[i].height;
        msg.objects.push_back(obj);

        ROS_INFO("classid-%d, %d", i, msg.objects[i].classid);
      }
    
      chatter_pub.publish(msg);
    }
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};



int main(int argc, char **argv)
{
  ros::init(argc,argv,"talker");
  
  ros::NodeHandle node_;
  std::string mode_file;
  std::string weights_file;
  std::string image_save;
  node_.param("model_file", mode_file, std::string("/home/riseauto/code/clCaffe/models/yolo/yolo416/fused_yolo_deploy.prototxt"));
  node_.param("weights_file", weights_file, std::string("/home/riseauto/code/clCaffe/models/yolo/yolo416/fused_yolo.caffemodel")); 
  node_.param("image_save", image_save, std::string("/tmp")); 

  pDetector = new Detector(mode_file, weights_file);
  ImageConverter ic;
  
  //ros::Publisher chatter_pub = node_.advertise<yolo_detect::ObjectVec>("chatter", 1000);
 
  ros::Rate loop_rate(10);

  //int count = 0;
  while (ros::ok())
  {
  #if 0 
    yolo_detect::ObjectVec msg;
    yolo_detect::Object obj;
    
    obj.classid = count;
    obj.confidence = 0.9;
    obj.rect.top = 50;
    obj.rect.left = 50;
    obj.rect.width = 100;
    obj.rect.height = 100;
    msg.objects.push_back(obj);

    obj.classid = count*100;
    obj.confidence = 0.9;
    obj.rect.top = 40;
    obj.rect.left = 40;
    obj.rect.width = 100;
    obj.rect.height = 100;
    msg.objects.push_back(obj);
    
    ROS_INFO("%d, %d", msg.objects[0].classid, msg.objects[1].classid);
  #endif
    //chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

    //++count;
  }
  
  return 0;
}

std::string get_object_classic(int class_id)
{
  std::string classic;
  switch(class_id)
  {
  case 0:
    classic = "aeroplane";
    break;
  case 1:
    classic = "bicycle";
    break;
  case 2:
    classic = "bird";
    break;
  case 3: 
    classic = "boat";
    break;
  case 4:
    classic = "bottle";
    break;
  case 5:
    classic = "bus";
    break;
  case 6:
    classic = "car";
    break;
  case 7:
    classic = "cat";
    break;
  case 8:
    classic = "chair";
    break;
  case 9:
    classic = "cow";
    break;
  case 10: 
    classic = "diningtable";
    break;
  case 11:
    classic = "dog";
    break;
  case 12:
    classic = "horse";
    break;
  case 13:
    classic = "motorbike";
    break;
  case 14:
    classic = "person";
    break;
  case 15:
    classic = "pottedplant";
    break;
  case 16:
    classic = "sheep";
    break;
  case 17: 
    classic = "sofa";
    break;
  case 18:
    classic = "train";
    break;
  case 19:
    classic = "tvmonitor";
    break;
  case 20:
    classic = "unknown";
    break;
  }
  return classic;
}
