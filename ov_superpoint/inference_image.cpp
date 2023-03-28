#include <memory>
#include <chrono>
#include "utils.h"
#include "super_glue.h"
#include "super_point.h"
#include "Hungarian.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>

using namespace cv;
using namespace std;

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

    superpoint->infer(image0, feature_points0);
    superpoint->infer(image1, feature_points1);

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

    Eigen::Matrix<double,Eigen::Dynamic, 256> fp0 = feature_points0.block(3, 0, 256, feature_points0.cols()).transpose();
    Eigen::Matrix<double,Eigen::Dynamic, 256> fp1 = feature_points1.block(3, 0, 256, feature_points1.cols()).transpose();
//    cout << fp0.rows() << fp0.cols();

    cv::Mat desc0, desc1;
    cv::eigen2cv(fp0, desc0);
    cv::eigen2cv(fp1, desc1);

    int num_src = feature_points0.cols();
    int num_tar = feature_points1.cols();
    Mat norm_self_0 = Mat::zeros(cv::Size(num_src, num_src), CV_64FC1);
    Mat norm_self_1 = Mat::zeros(cv::Size(num_tar, num_tar), CV_64FC1);
    Mat norm_cross = Mat::zeros(num_src, num_tar, CV_64FC1);

    for(int i=0; i<num_src; i++){
        for(int j=0; j<num_src; j++){
            norm_self_0.at<double>(i,j) =cv::norm(desc0.row(i),desc0.row(j),NORM_L2);
        }}

    for(int i=0; i<num_tar; i++){
        for(int j=0; j<num_tar; j++){
            norm_self_1.at<double>(i,j) =cv::norm(desc1.row(i),desc1.row(j),NORM_L2);
        }}

    vector<vector<double>> cost_matrix = vector<vector<double> >(num_src, vector<double>(num_tar));

    for(int i=0; i<num_src; i++){
        for(int j=0; j<num_tar; j++){
            double rs = cv::norm(desc0.row(i),desc1.row(j),NORM_L2);
//            norm_cross.at<double>(i,j) = rs;
            cost_matrix[i][j] = rs;
        }
    }
//    norm_cross = cv::Mat(num_src,num_tar,CV_64FC1, cost_matrix.data());
//    superglue->matching_points(feature_points0, feature_points1, superglue_matches);

    HungarianAlgorithm HungAlgo;
    vector<int> assignment;
    double cost = HungAlgo.Solve(cost_matrix, assignment);
    std::vector<cv::DMatch> hungarian_matches;

    for (int i=0; i < assignment.size(); i++){
        int tar_id = assignment[i];
        if(tar_id != -1){
            hungarian_matches.push_back(DMatch(i,tar_id,cost_matrix[i][tar_id]));
        }
    }

    cv::Mat match_image;
    cv::drawMatches(image0, keypoints0, image1, keypoints1, hungarian_matches, match_image);
    cv::imshow("match", match_image);
    cv::waitKey();

/* Task list:
 * - Major: Implement superpoint - sinkhorn - hungarian
 * - Minor: Implenent PnP-solver ? - Stereo
 * */
//    cv::destroyAllWindows();
    return 0;
}