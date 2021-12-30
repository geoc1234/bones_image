#include "myFace.hpp"

float scale = 1.0;
float scoreThreshold = 0.9;
float nmsThreshold = 0.3;
int topK = 5000;
double cosine_similar_thresh = 0.363;
double l2norm_similar_thresh = 1.128;
std::string fd_modelPath = "/home/geo/models/yunet.onnx";
cv::Ptr<cv::FaceDetectorYN> detector = cv::FaceDetectorYN::create(fd_modelPath, "", cv::Size(320, 240), scoreThreshold, nmsThreshold, topK);
std::string fr_modelPath = "/home/geo/models/face_recognizer_fast.onnx";
cv::Ptr<cv::FaceRecognizerSF> faceRecognizer = cv::FaceRecognizerSF::create(fr_modelPath, "");


bool getTargetProfile(cv::Mat &inputImage, cv::Mat &targetProfile, bool verbose)
{
    cv::Mat faces;
    int frameHeight = inputImage.rows;
    int frameWidth = inputImage.cols;
    cv::Mat im;
    //detector->setInputSize(cv::Size(frameWidth, frameHeight));
    cv::resize(inputImage,im,cv::Size2i(320,240));
    int fcs = detector->detect(im, faces);
    if(fcs != 1)
    {
        if(verbose) std::cout << "face not found in profile pic";
        return false;
    }
    cv::Mat aligned_face;
    cv::Mat features;
    faceRecognizer->alignCrop(im, faces.row(0), aligned_face);
    faceRecognizer->feature(aligned_face, features);
    targetProfile = features.clone();
    return true;
}

bool isTargetPresent(cv::Mat &inputImage, cv::Mat &targetProfile, cv::Rect2i &box, bool verbose)
{
    cv::Mat faces;
    cv::Mat im;
    cv::resize(inputImage,im,cv::Size2i(320,240));

    int fcs = detector->detect(im, faces);
    if(fcs == 0) return false;

    cv::Mat aligned_face;
    cv::Mat features;
    cv::Mat profile;
    for(int i = 0; i < faces.rows; i++){
        faceRecognizer->alignCrop(im, faces.row(i), aligned_face);
        faceRecognizer->feature(aligned_face, features);
        profile = features.clone();
        try{
            double cos_score = faceRecognizer->match(targetProfile, profile, cv::FaceRecognizerSF::DisType::FR_COSINE);
            double L2_score = faceRecognizer->match(targetProfile, profile, cv::FaceRecognizerSF::DisType::FR_NORM_L2);
            if(cos_score > cosine_similar_thresh && L2_score < l2norm_similar_thresh){
                box.x = int(faces.at<float>(i, 0));
                box.y = int(faces.at<float>(i, 1));
                box.width = int(faces.at<float>(i, 2));
                box.height = int(faces.at<float>(i, 3));
                return true;    
            }    
        }        
        catch(const std::exception& e){
            std::cerr << e.what() << '\n';
        }     
    }   
    return false;
}
