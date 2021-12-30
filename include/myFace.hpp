#ifndef MY_FACE
#define MY_FACE

#include <opencv4/opencv2/objdetect/face.hpp>
#include <opencv4/opencv2/face/facerec.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

extern float scale;
extern float scoreThreshold;
extern float nmsThreshold;
extern int topK;
extern double cosine_similar_thresh;
extern double l2norm_similar_thresh;
extern std::string fd_modelPath;
extern cv::Ptr<cv::FaceDetectorYN> detector;
extern std::string fr_modelPath;
extern cv::Ptr<cv::FaceRecognizerSF> faceRecognizer;

bool getTargetProfile(cv::Mat &inputImage, cv::Mat &targetProfile, bool verbose);

bool isTargetPresent(cv::Mat &inputImage, cv::Mat &targetProfile, cv::Rect2i &box, bool verbose);


#endif