#include "ros/ros.h"
#include "std_msgs/String.h"
#include "start/Rect.h"
#include "start/Object.h"
#include "start/ObjectVec.h"

#include <sstream>

//#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    //image_sub_ = it_.subscribe("/camera/image_raw", 1,
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
      ROS_INFO("cv_bridge info");
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      ROS_INFO("cv_bridge info");
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

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
  ImageConverter ic;
  //ros::spin();
  
  ros::NodeHandle n;

  //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Publisher chatter_pub = n.advertise<start::ObjectVec>("chatter", 1000);
 
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    
    /*std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    */
    
    start::ObjectVec msg;
    start::Object obj;
    
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

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

    ++count;
  }
  
  return 0;
}
