#include <memory>
#include <chrono>
#include "utils.h"
#include "super_glue.h"
#include "super_point.h"

//#include <opencv2/imgproc/imgproc.hpp>
//#include <vpi/OpenCVInterop.hpp>
//#include <vpi/Array.h>
//#include <vpi/Image.h>
//#include <vpi/Pyramid.h>
//#include <vpi/Status.h>
//#include <vpi/Stream.h>
//#include <vpi/algo/ConvertImageFormat.h>
//#include <vpi/algo/GaussianPyramid.h>
//#include <vpi/algo/HarrisCorners.h>
//#include <vpi/algo/OpticalFlowPyrLK.h>

using namespace cv;

int main()
{
    std::string folder = "/home/hoangqc/Datasets/TartanAir/office/Easy/P004/";
    std::string config_path = "/home/hoangqc/Datasets/Weights/config.yaml";
    std::string model_dir = "/home/hoangqc/Datasets/Weights/";

    Configs configs(config_path, model_dir);

    int step = 5;
    cv::Mat img_l, img_r, image0, image1;
    std::vector<std::string> image_left, image_right;
    GetFileNames(folder+"image_left", image_left);
    GetFileNames(folder+"image_right", image_right);
    image0 = cv::imread(image_left[160], IMREAD_GRAYSCALE);
    image1 = cv::imread(image_left[180], IMREAD_GRAYSCALE);
//    img_l = cv::imread(image_left[160], IMREAD_GRAYSCALE);
//    img_r = cv::imread(image_right[160], IMREAD_GRAYSCALE);
//    std::cout << "First image size: " << img_l.cols << "x" << img_l.rows << std::endl;
//    std::cout << "Second image size: " << img_r.cols << "x" << img_r.rows << std::endl;

    std::cout << "Building inference engine......" << std::endl;
    auto superpoint = std::make_shared<SuperPoint>(configs.superpoint_config);
    std::cout << configs.superpoint_config.onnx_file << std::endl << configs.superglue_config.onnx_file << std::endl;
    if (!superpoint->build()){
        std::cerr << "Error in SuperPoint building engine. Please check your onnx model path." << std::endl;
        return 0;
    }

    auto superglue = std::make_shared<SuperGlue>(configs.superglue_config);
    if (!superglue->build()){
        std::cerr << "Error in SuperGlue building engine. Please check your onnx model path." << std::endl;
        return 0;
    }

    std::cout << "SuperPoint and SuperGlue inference engine build success." << std::endl;
    Eigen::Matrix<double, 259, Eigen::Dynamic> feature_points0, feature_points1;
    std::vector<cv::DMatch> superglue_matches;
    std::cout << "SuperPoint and SuperGlue test." << std::endl;
    std::cout << "---------------------------------------------------------" << std::endl;
    if(!superpoint->infer(image0, feature_points0)){
        std::cerr << "Failed when extracting features from first image." << std::endl;
        return 0;
    }
    if(!superpoint->infer(image1, feature_points1)){
        std::cerr << "Failed when extracting features from second image." << std::endl;
        return 0;
    }
    superglue->matching_points(feature_points0, feature_points1, superglue_matches);

    cv::Mat match_image;
    std::vector<cv::KeyPoint> keypoints0, keypoints1;
    for(size_t i = 0; i < feature_points0.cols(); ++i){
        double score = feature_points0(0, i);
        double x = feature_points0(1, i);
        double y = feature_points0(2, i);
        keypoints0.emplace_back(x, y, 8, -1, score);
    }
    for(size_t i = 0; i < feature_points1.cols(); ++i){
        double score = feature_points1(0, i);
        double x = feature_points1(1, i);
        double y = feature_points1(2, i);
        keypoints1.emplace_back(x, y, 8, -1, score);
    }
    cv::drawMatches(image0, keypoints0, image1, keypoints1, superglue_matches, match_image);

    cv::imshow("match", match_image);
    cv::waitKey();

//    for(int i = 0; i< image_left.size()/step;i++)
//    {
//
//        img_l = cv::imread(image_left[i*step], IMREAD_GRAYSCALE);
//        img_r = cv::imread(image_right[i*step], IMREAD_GRAYSCALE);
//        imshow("img_l", img_l);
//        imshow("img_r", img_r);
//        if (waitKey() == 'q')
//            break;
//    }

/* Task list:
 * - Major: Implement superpoint - sinkhorn - hungarian
 * - Minor: Implenent PnP-solver ? - Stereo
 * */
    cv::destroyAllWindows();
    return 0;
}