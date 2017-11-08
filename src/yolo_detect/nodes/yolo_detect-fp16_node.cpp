#include "yolo_detect/yolo_detect-fp16.h" 
#include "ruimou_vision/object_tracking/MultiObjectTracker.h"

#include <chrono>
#include <opencv2/highgui/highgui.hpp>

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



using namespace ruimou;

static double now()
{
    using namespace std::chrono;

    system_clock::duration epoch = system_clock::now().time_since_epoch();

    return std::chrono::duration_cast
        <std::chrono::duration<double, std::ratio<1, 1>>>(epoch).count();
}


static const std::string OPENCV_WINDOW = "Image window";
static std::string get_object_classic(int class_id);

class ImageHandler
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
  ros::Publisher chatter_pub;
  
  MultiObjectTracker tracker;

  std::vector<cv::Rect > rects;
  std::vector<int > class_id;
  std::vector<float > confidences;
  Detector yolo_detector;

  cv_bridge::CvImagePtr cv_ptr;

public:
  ImageHandler(const ros::NodeHandle& nh, std::string model_file, std::string weights_file):it_(nh),yolo_detector(model_file, weights_file)
  //ImageHandler(const Detector& detector, const ros::NodeHandle& nh)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
      &ImageHandler::imageHandleCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
      cv::namedWindow(OPENCV_WINDOW);
    
    nh_ = nh;
    //it_ = image_transport::ImageTransport(nh);
    //yolo_detector(detector);
    chatter_pub = nh_.advertise<yolo_detect::ObjectVec>("chatter", 1000);
  }

  ~ImageHandler()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  bool imageConvert(const sensor_msgs::ImageConstPtr& msg)
  {
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return false;
    }
    return true;
  }
  
  void do_yolo_detect()
  {
    rects.clear();
    class_id.clear();
    confidences.clear();
    yolo_detector.Detect(cv_ptr->image, rects, class_id, confidences);

    #if 1
    yolo_detect::ObjectVec objmsg;
    yolo_detect::Object obj;
   
    //objmsg.header = msg->header;
    ROS_INFO("-------------time: %u", objmsg.header.seq);
    for(int i=0; i<rects.size(); i++)
    {
      obj.classid = class_id[i];
      obj.confidence = confidences[i];
      obj.rect.top = rects[i].x;
      obj.rect.left = rects[i].y;
      obj.rect.width = rects[i].width;
      obj.rect.height = rects[i].height;
      objmsg.objects.push_back(obj);

      //ROS_INFO("classid-%d, %d", i, msg.objects[i].classid);
      cv::rectangle(cv_ptr->image, rects[i], cv::Scalar(255,255, 0), 1, 1, 0);
      char info[255] = {0};
      snprintf(info, sizeof(info), "Conf:%f", obj.confidence);
      cv::putText(cv_ptr->image, info,
                  cvPoint(rects[i].x, rects[i].y+20),
                  CV_FONT_HERSHEY_DUPLEX, 0.6f, CV_RGB(255, 0, 0));
    }
    chatter_pub.publish(objmsg);
    #endif
  }
  
  void do_object_track()
  {
    #if 1 
    double curTime = now();
    std::vector<Tracking> trackings = tracker.update(cv_ptr->image, rects, curTime);
    for (size_t i = 0; i < trackings.size(); i++) 
    {
      if(trackings[i].state == Tracker::TRACKED)
      {
        cv::rectangle(cv_ptr->image, trackings[i].bbox, cv::Scalar(0,255, 0), 1, 1, 0);
        #if 1
        char userDescription[255] = {0};
        snprintf(userDescription, sizeof(userDescription), "ID:%d", trackings[i].id);
        cv::putText(cv_ptr->image, userDescription,
                  cvPoint(trackings[i].bbox.x, trackings[i].bbox.y),
                  CV_FONT_HERSHEY_DUPLEX, 0.6f, CV_RGB(255, 0, 0));
        #endif
      }
    }
    #endif
  }
  
  void imageHandleCb(const sensor_msgs::ImageConstPtr& msg)
  {
    if(imageConvert(msg))
    {
       do_yolo_detect();
       do_object_track();
    }

    cv::Mat dst;
    cv::resize(cv_ptr->image, dst, cv::Size(cv_ptr->image.cols*2, cv_ptr->image.rows*2), 0, 0, cv::INTER_LINEAR);

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, dst);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char **argv)
{
  ros::init(argc,argv,"talker");
  
  ros::NodeHandle node_;
  ros::NodeHandle nh("~");
  
  std::string model_file;
  std::string weights_file;
  std::string image_save;
  if(!nh.getParam("model_file", model_file))
    nh.param("model_file", model_file, 
             std::string("/home/riseauto/code/clCaffe/models/yolo/yolo416/fused_yolo_deploy.prototxt"));
  if(!nh.getParam("weights_file", weights_file))
    nh.param("weights_file", weights_file, 
             std::string("/home/riseauto/code/clCaffe/models/yolo/yolo416/fused_yolo.caffemodel")); 
  if(!nh.getParam("image_save", image_save))
    nh.param("image_save", image_save, std::string("/tmp")); 


  //Detector yolo_detector(node_, model_file, weights_file);
  
  ImageHandler ic(node_, model_file, weights_file);
 
  ros::spin();
  
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
