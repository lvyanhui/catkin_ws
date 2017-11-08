#ifndef __YOLO_DETECT_FP16_H__
#define __YOLO_DETECT_FP16_H__

#include <caffe/caffe.hpp>
#ifdef USE_OPENCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#endif  // USE_OPENCV
#include <algorithm>
#include <iomanip>
#include <iosfwd>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#if defined(USE_OPENCV) && defined(HAS_HALF_SUPPORT)
using namespace caffe;  // NOLINT(build/namespaces)

#define Dtype half
#define RETRIVE_MSG

class Detector {
 public:
  //Detector(){};
  Detector(const string& model_file,
           const string& weights_file);

  void Detect(cv::Mat& img);
  void Detect(cv::Mat& img, std::vector<cv::Rect > &rects, std::vector<int > &class_id, std::vector<float > &confidences);
  int model_file;

 private:
  void WrapInputLayer(std::vector<Dtype *> &input_channels);

  void Preprocess(const cv::Mat& img,
                  std::vector<Dtype *> input_channels);
 
  shared_ptr<Net<Dtype> > net_;
  cv::Size input_geometry_;
  cv::Size input_newwh_;  
  int num_channels_;
};

#else
#endif  // USE_OPENCV
#endif  
